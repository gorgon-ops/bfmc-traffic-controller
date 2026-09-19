# The broadcast

What the semaphore tells the cars, and what the cars are entitled to believe.

Everything here is produced by `firmware/common/app/sem_proto.c`. It is the
only file in the project that knows the payload is text, let alone that it is
JSON.

---

## The frame

One UDP datagram per head, ten times a second:

```json
{"id":3,"state":2,"ttc_ms":4200,"exact":1}
```

| key | meaning |
|---|---|
| `id` | which head. One per approach, not one per intersection — a four-way junction broadcasts four ids |
| `state` | `0` red, `1` yellow, `2` green |
| `ttc_ms` | milliseconds until this state can next change |
| `exact` | `1` when `ttc_ms` is the answer, `0` when it is only a lower bound |
| `fault` | present, and `1`, only when the intersection is in fault flash. Absent otherwise |

`id` and `state` are the two required keys and are the BFMC pair. The rest are
additive: a consumer that parses the object into a dictionary and reads two
fields out of it is unaffected by their presence.

> **Check this against your rulebook.** The official payload has moved between
> editions of the competition. If it has moved again, it has moved in
> `sem_proto_encode()` and nowhere else — nothing above that function knows
> what the bytes look like. `sem_proto_decode()` is the matching reader and is
> what `tests/test_proto.c` round-trips against.

---

## Two decisions worth arguing with

### The pre-green is broadcast as red

The European sequence lights red and yellow together for 800 ms before green.
`state` reports that as `0`, red.

The field answers *may I enter*, not *which bulbs are lit*. A car that saw
`state: 1` during the pre-green and treated it as a yellow it could run would
be entering the intersection on a red, against traffic that still has its own
clearance running. If you want the bulbs, that is a different field and this
protocol does not carry one.

### `ttc_ms` is the earliest the state can change, never the latest

Green is the only interval whose length is not fixed. Under actuation it is
extended by cars that have not arrived yet, so at the moment a frame is built
there is no honest countdown to the end of it — only a floor.

So the rule is one rule, in both directions:

- **green** — `ttc_ms` is the guaranteed remaining green. It will not end
  sooner. It may last longer.
- **red** — `ttc_ms` is the earliest the green can arrive. It will not arrive
  sooner. It may take longer.
- **yellow** — always exact. Yellow is never extended, by anything.

`exact` is `1` when nothing between now and the change can be extended or
skipped, which on a fixed-time plan is always. `tests/test_proto.c` checks this
the only way it can be checked: read the frame, run the intersection forward,
and compare against when the state actually changed — on every sample across
several minutes of both plans.

The alternative to publishing a bound is publishing a number that is sometimes
wrong, and a planner that trusts a wrong countdown enters the box on it.

### When the unit is faulted

```json
{"id":1,"state":0,"ttc_ms":0,"exact":0,"fault":1}
```

`state` is red because the heads are flashing red. `ttc_ms` is zero with
`exact` clear, which in the rule above reads as *it can change at any time and
nothing is promised* — which is the truth. `fault` is what distinguishes this
from a red with a countdown attached. A car that sees it should treat the
intersection as unsignalised: stop, then proceed when the box is clear.

---

## Consuming it

```python
import json, socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5007))

heads = {}
while True:
    data, _ = sock.recvfrom(512)
    for line in data.decode().splitlines():
        f = json.loads(line)
        heads[f["id"]] = f
```

`tools/listen.py` is that, plus a table. Point it at a laptop running

```bash
build/host/sem_sim --broadcast --addr=127.0.0.1
```

and you have the full cycle, the pedestrian phases and the countdowns on the
wire without a track, a board or an intersection.

### Deciding to stop

The useful shape on the car side, with `v` the approach speed and `a_max` a
comfortable deceleration:

```
t_to_stopline = distance / v

state == GREEN and exact and ttc_ms/1000 > t_to_stopline   -> go, it is still green when you arrive
state == GREEN and not exact                               -> go only if ttc_ms/1000 > t_to_stopline
                                                              (the bound is the guarantee; anything
                                                               beyond it is luck)
state == YELLOW                                            -> ttc_ms is exact. Stop unless you are
                                                              already inside the dilemma zone
state == RED                                               -> ttc_ms is the earliest green. Plan the
                                                              deceleration against it, do not launch on it
fault present                                              -> unsignalised. Stop and give way
```

The one thing never to do is launch on a red whose `ttc_ms` has run to zero.
Zero on a red means *not before now*, not *now*.

---

## Rate and transport

Ten frames a second per head, each its own datagram, UDP broadcast, no
acknowledgement and no sequence number. A dropped frame costs a car 100 ms of
staleness and the next one fixes it; a protocol that retransmitted would be
spending effort to deliver an old answer.

On the node the rate comes off the tick timer and is as steady as the timer.
The host broadcaster in `sim/` sleeps on a desktop scheduler instead, so it
lands nearer 9 Hz than 10 — good enough for developing against, and not what
the hardware will do.

Frames are sent from the main loop, never from the tick interrupt, and are
dropped rather than queued if the link cannot keep up. A semaphore whose lamp
timing depended on how fast a radio module was feeling would be a worse
semaphore than one that occasionally misses a frame.
