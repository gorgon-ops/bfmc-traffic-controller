# A traffic signal controller for the Bosch Future Mobility Challenge

The track-side half of the problem. Not the car — the intersection: a state
machine that decides which approaches get a green, a conflict monitor that
watches the lamp outputs and does not trust it, and a broadcast that tells the
cars what they are looking at and how long they have.

**Status:** controller, monitor and protocol complete and verified in host
simulation. Board layer written and type-checked against a stubbed HAL;
hardware has not been built. `docs/bringup-checklist.md` is the order to find
out in.

---

## The actual problem

A traffic light is four timers, which is why it looks like a first-year
exercise. What makes it an engineering problem is that the timers are not the
part that can hurt anybody.

Two approaches green at once is the failure. Everything else — the phase
sequence, the actuation, the pedestrian timings — is machinery for deciding
*when* to give a green, and all of it can be perfectly correct while the
intersection shows something lethal, because the thing that puts a green into
the road is an output driver and the state machine cannot see output drivers.
A shorted transistor, a connector on the wrong head, a conflict matrix copied
from a different junction, firmware that has stopped running with its outputs
latched: in every one of those the controller is behaving flawlessly.

That is why a real signal cabinet contains two computers. The second one, the
conflict monitor, watches the voltage on the lamp wires and drops the
intersection to flashing red when what it sees is unsafe, regardless of what
the first one believes it commanded. This project is built around that split,
and `docs/conflict.md` is honest about where the split stops being real on a
single-processor board.

The second problem is what to tell the cars. A BFMC semaphore broadcasts its
state over the network, and a car that knows only *red* or *green* has to
approach every intersection as though the light might change on the next
frame. What it wants is the countdown. The awkward part is that under
actuation the countdown does not exist — a green is extended by cars that have
not arrived yet — so the honest thing to publish is a bound, clearly labelled
as one:

> **`ttc_ms` is the earliest the state can change, never the latest.** `exact`
> says which of the two it is.

A green will not end sooner than the number says. A red will not clear sooner
than the number says. Neither promises anything about later. A planner that
trusts a lying countdown enters the box on it, and the alternative to a bound
is a number that is right most of the time.

---

## Results

### What the conflict monitor catches

Each fault injected into the **field readback** — downstream of the state
machine, so the controller is behaving correctly throughout. `make faults`.

| injected | detected as | time to flash |
|---|---|---|
| shorted driver, green on a conflicting approach | conflicting greens, groups 0 and 1 | same tick |
| lamp failed open, head dark | field output does not match command, group 3 | 100 ms, one settling window |
| field drops yellow after 500 ms | yellow too short, group 0 | same tick |
| green appears 300 ms into the all-red | all-red too short, groups 1 and 0 | same tick |
| controller stops calling the monitor | controller stopped reporting | first call after a gap over 250 ms |

The heads go safe on the tick the fault is found, not the next one, and the
fault latches — an intersection that recovers from a conflict on its own will
show you that conflict again and never tell you it happened.

### What actuation is worth

Mean delay per car, from joining the queue to crossing the stop line. Twenty
simulated minutes per case, five seeds, against the same junction running a
fixed cycle. `make sim`.

**Four-way, demand even on all four approaches**

| cars/min/approach | fixed mean | fixed worst | actuated mean | actuated worst | gain |
|---|---|---|---|---|---|
| 6 | 5.1 s | 15.2 s | **4.8 s** | 13.7 s | +8% |
| 12 | 5.7 s | 31.0 s | **5.5 s** | 18.6 s | +5% |
| 18 | 8.8 s | 50.0 s | **6.7 s** | 26.9 s | +25% |
| 24 | 49.4 s | 160.9 s | **9.9 s** | 43.4 s | +80% — fixed time is over capacity here |

**T junction, main road at 18/min each way, stem as shown**

| stem cars/min | fixed mean | actuated mean | gain |
|---|---|---|---|
| 1 | 5.96 s | **2.69 s** | +55% |
| 4 | 5.96 s | **4.63 s** | +23% |
| 12 | 6.43 s | **5.53 s** | +15% |

The even four-way is the case where actuation has least to offer, and the
numbers say so: at low demand a fixed plan is already nearly right and the gain
is inside the noise. It pays where the demand is lopsided — the T junction with
an empty stem — and it pays where a fixed plan runs out of capacity, because
resting on the busy approach is capacity a fixed cycle cannot reach.

### Verification

| | |
|---|---|
| Host assertions | **925,211**, no hardware, ~15 s |
| Longest continuous run checked tick-by-tick | 40 simulated minutes across four plans |
| Independent invariants checked every tick | conflicts, pedestrian conflicts, aspect sequence, yellow length, all-red measured from the field, broadcast agreement, starvation |
| Countdown claim | verified by reading the frame, running a copy of the intersection forward, and comparing against the real change — exact to the tick on fixed plans, never early on actuated ones |
| Fault classes injected and caught | 5 |
| Control tick | 100 Hz |
| Broadcast | 10 Hz per head |

Reproduce with `make`.

---

## Try it without hardware

```bash
make timeline
```

Every change of indication for a minute, with the frame that went out with it:

```
    time |  1  2  3  4 | x0 x1 | state     | frame for id 1
  -------+-------------+-------+-----------+----------------------------------------
    0.0s | *R *R *R *R |       | startup   | {"id":1,"state":0,"ttc_ms":4790,"exact":1}
    3.0s | R  R  R  R  |  .  . | all-red   | {"id":1,"state":0,"ttc_ms":1800,"exact":1}
    4.0s | RY R  RY R  |  .  . | pre-green | {"id":1,"state":0,"ttc_ms":800,"exact":1}
    4.8s | G  R  G  R  |  W  . | green     | {"id":1,"state":2,"ttc_ms":4000,"exact":0}
    6.3s | G  R  G  R  |  c  . | green     | {"id":1,"state":2,"ttc_ms":2500,"exact":0}
    8.3s | G  R  G  R  |  .  . | green     | {"id":1,"state":2,"ttc_ms":500,"exact":0}
    8.8s | Y  R  Y  R  |  .  . | yellow    | {"id":1,"state":1,"ttc_ms":1500,"exact":1}
   10.3s | R  R  R  R  |  .  . | all-red   | {"id":1,"state":0,"ttc_ms":9100,"exact":0}
```

The pre-green at 4.0 s is the European red-and-yellow before green. Note that
the frame still says `"state":0` — the field answers *may I enter*, not *which
bulbs are lit*.

And for developing the car side before the track exists:

```bash
build/host/sem_sim --broadcast        # real UDP, real time, 10 Hz per head
python tools/listen.py                # the other end
```

---

## How it is put together

```
firmware/common/app/     portable C99, no HAL, runs identically on host and target
  lamps.c                aspects to bulb vectors, flash resolved from the clock
  intersection.c         the phase state machine, and the countdown
  intersection_config.h  every timing number, with its reason
  plans.c                the intersections on the track, as data
  conflict_monitor.c     the part that does not trust the state machine
  sem_proto.c            the wire format, and the only file that knows it is text
  semaphore.c            one unit behind one call

firmware/semaphore_node/ the STM32 layer. Pins, UART, tick. Nothing decides here.
sim/                     traffic, a clock, and a real UDP socket
tests/                   five programs, 925k assertions
tools/hal_stub/          just enough HAL to compile the board layer on a PC
docs/                    protocol, conflict monitor, hardware, testing, bring-up
```

The division that matters: nothing in `firmware/common/app` includes a HAL
header or a socket header. That is why the same code that will run on the node
runs under `tests/` on a PC and under `sim/` against simulated traffic, and why
the timeline above came out of the same `intersection_update()` that will drive
the lamps.

An intersection is **data**. Adding one is a table in `plans.c` — the
approaches, which of them conflict, the stages, the broadcast ids — and
`intersection_plan_check()` refuses the table if the edit made it unsafe: an
asymmetric conflict matrix, a stage that greens two approaches that conflict,
an approach no stage ever serves, a pedestrian clearance longer than the green
that has to contain it. It runs at boot on the node too, and a plan that fails
leaves the unit in flashing red with the reason on the debug UART rather than
guessing at a configuration it cannot verify.

---

## Adapting it to your competition

Two things will need attention:

**The wire format.** The official BFMC payload has moved between editions. The
`id` and `state` pair is the part to check against your year's rulebook and
startup code, and it lives in `sem_proto_encode()` — one function, and nothing
above it knows the payload is JSON or even text. `sem_proto_decode()` and
`tests/test_proto.c` round-trip against it.

**The timings.** Everything in `intersection_config.h` is scaled for a 1:10
track, where a car crosses an intersection in about a second. The structure is
not scaled — minimum green, a fixed yellow, an all-red clearance between
conflicting movements all exist for the same reasons they do full size. The one
number not to shorten casually is the all-red: it covers a car that entered
late on yellow, and on this track that car is under closed-loop control and may
be braking hard for a pedestrian doll it has just detected.

---

## What is not done

- **The monitor shares a core with the controller.** In a cabinet it is a
  separate board on separate power. Here a controller that hard-faults with a
  green latched takes the monitor down with it. Closing it properly is a second
  microcontroller holding the conflict matrix and driving the lamp supply
  through an AND with the main processor's watchdog — a board revision, and the
  most worthwhile thing on this list. `docs/conflict.md`.
- **The readback cannot see a dead bulb.** It measures voltage at the
  processor, which catches a shorted driver and a miswired head but not a LED
  that has failed open. The sense-resistor version is drawn up in
  `docs/hardware.md` and not built.
- **No protected turns.** Turning movements are permissive, the way they are at
  the unsignalised junctions elsewhere on the track. The machinery supports
  splitting an approach into two signal groups with their own conflict rows;
  the plans simply do not use it.
- **No coordination between intersections.** Each unit runs its own cycle off
  its own clock, which is the honest thing to build when the units have no
  shared time base. A green wave needs one.
- **Never powered.** Everything above is simulation and a compiler.
