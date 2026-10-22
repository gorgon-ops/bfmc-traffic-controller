# A traffic signal controller for the Bosch Future Mobility Challenge

The track-side half of the problem. Not the car — the intersection: a state
machine that decides which approaches get a green, a conflict monitor that
watches the lamp outputs and does not trust it, and a broadcast that tells the
cars what they are looking at and how long they have.

**Status:** controller, monitor and protocol complete and verified in host
simulation. The board layer is written and now runs on the host against a fake
HAL; it has never been on hardware. `docs/bringup-checklist.md` is the order to
find out in.

```bash
make            # everything: ~36 s, no hardware
```

> On Windows with MinGW the binary is `mingw32-make` and there is no `make` on
> PATH — every `make X` below means `mingw32-make X` there. CI runs GNU make on
> Linux, which is what the instructions assume.

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

| injected | detected as | time to flashing red |
|---|---|---|
| shorted driver, green on a conflicting approach | conflicting greens, groups 0 and 1 | same tick |
| lamp failed open, head dark | field output does not match command, group 3 | 100 ms, one settling window |
| field drops yellow after 500 ms | yellow too short, group 0 | same tick |
| green appears 300 ms into the all-red | all-red too short, groups 1 and 0 | same tick |
| controller stops calling the monitor | controller stopped reporting | first call after a gap over 250 ms |

The heads go safe on the tick the fault is found, not the next one, and the
fault latches — an intersection that recovers from a conflict on its own will
show you that conflict again and never tell you it happened.

The same five faults are injected through the **board layer** in
`tests/test_node.c`, where a jammed output pin has to end with the lamp supply
relay dropped as well as the pattern flashing.

### What actuation is worth

Mean delay per car, from joining the queue to crossing the stop line. Twenty
simulated minutes per case, five seeds. `make sim`.

The comparison is against **Webster's 1958 formula**, retimed for each demand
level — cycle length `C = (1.5L + 5)/(1 − Y)`, split in proportion to each
stage's flow ratio, with the practical minimum green a traffic engineer would
apply by hand afterwards. The "naive fixed" column is the single six-second
green this project started with, kept in because the difference between the
two columns is the point: measuring against a badly timed baseline is how you
get an impressive number that means nothing.

**Four-way, demand even on all four approaches**

| cars/min | naive fixed | Webster cycle | Webster mean | actuated mean | actuated worst | gain vs Webster |
|---|---|---|---|---|---|---|
| 6 | 5.1 s | 19.5 s | 5.1 s | **4.8 s** | 13.7 s | +7% |
| 12 | 5.7 s | 24.1 s | 6.0 s | **5.5 s** | 18.6 s | +10% |
| 18 | 8.8 s | 31.5 s | 8.3 s | **6.7 s** | 26.9 s | +20% |
| 24 | over capacity | 45.6 s | 12.0 s | **9.9 s** | 43.4 s | +18% |

**T junction, main road at 18 cars/min each way**

| stem cars/min | Webster mean | Webster worst | actuated mean | actuated worst | gain vs Webster |
|---|---|---|---|---|---|
| 1 | 3.5 s | 20.5 s | **2.7 s** | 17.1 s | +24% |
| 4 | 4.2 s | 44.9 s | 4.6 s | **18.6 s** | −8% on the mean |
| 12 | 6.4 s | 33.0 s | **5.5 s** | 21.0 s | +15% |

Against a properly timed fixed plan the gain is 7–24%, not the 80% the naive
baseline suggested. The row worth reading is the T junction at four cars a
minute on the stem, where actuation is **8% worse on the mean and 59% better
on the worst case** — a fixed plan that serves an almost-empty stem every
cycle has a good average and a long tail, and the tail is what a competition
run is actually judged on.

### Verification

| | |
|---|---|
| Host assertions | **925,389** across seven programs, no hardware, ~36 s |
| Line coverage, portable core | **92.4%** of 646 lines (`make coverage`) |
| Longest run checked tick-by-tick | 40 simulated minutes across four plans |
| Independent invariants, every tick | conflicts, pedestrian conflicts, aspect sequence, yellow length, all-red measured from the field, broadcast agreement, starvation |
| Countdown claim | verified by reading the frame, running a copy of the intersection forward, and comparing against the real change |
| Parser | differential-fuzzed against an independently written oracle; 27-case corpus plus a deterministic mutation sweep in `make test`, libFuzzer + ASan + UBSan in CI |
| Fault classes injected and caught | 5, in the core and again through the board layer |
| Delay regression guard | `make check` fails if the figures move more than 8% |
| Warnings | clean under `-Wconversion -Wsign-conversion -Wshadow -Wpedantic -Wcast-qual -Wdouble-promotion` and five more, as errors |

Per-program counts and what each one is for: `docs/testing.md`.

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
sim/                     traffic, a clock, a real UDP socket, Webster's formula
tests/                   seven programs; fake_hal.c is what lets the board layer run
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

## Limitations

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
- **The proof is bounded, and has not been run here.** `make prove` model-checks
  the no-conflicting-green invariant with CBMC over *all* call sequences to a
  depth of about forty ticks, which is deeper than a stage transition and
  shallower than an hour of operation. CBMC is not installed on the machine this
  was written on, so that job has only ever run in CI — see below.
- **`make fuzz`, `make arm` and the sanitizer build have only run in CI.** The
  development machine is Windows with MinGW: no clang, no libasan, no
  arm-none-eabi. The replay fuzzer and the CBMC harnesses have concrete builds
  that do run locally in `make test`, which is what keeps them from rotting, but
  the first green CI run is the first real evidence for the other three.
- **No footprint number yet.** `make arm` cross-compiles the core and the board
  layer and prints `arm-none-eabi-size`, but nobody has run it, so this README
  does not quote flash or RAM. The same goes for the worst-case tick, which the
  node measures with DWT and reports on the debug UART and which has no value
  until it has run on an ARM.
- **No protected turns.** Turning movements are permissive, the way they are at
  the unsignalised junctions elsewhere on the track. The machinery supports
  splitting an approach into two signal groups with their own conflict rows;
  the plans simply do not use it.
- **No coordination between intersections.** Each unit runs its own cycle off
  its own clock, which is the honest thing to build when the units have no
  shared time base. A green wave needs one.
- **Never powered.** A Blue Pill and twelve LEDs is about ten euros and would
  end this section; until then everything above is simulation and a compiler.
