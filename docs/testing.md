# Testing

```bash
make            # everything: the five test programs, the simulation, the board layer
make test       # just the test programs
```

925,211 assertions, no hardware, about fifteen seconds.

| program | assertions | what it is for |
|---|---|---|
| `test_plan` | 41 | the configuration check, driven with plans that are wrong on purpose |
| `test_phases` | 75 | the sequence, and the length of every interval in it |
| `test_monitor` | 869 | the conflict monitor, against field output no controller would produce |
| `test_proto` | 20,209 | the frame, and whether its countdown is ever early |
| `test_integration` | 904,017 | forty simulated minutes, every invariant checked on every tick |

The counts are lopsided on purpose. The first three are hand-written cases and
there are as many of them as there are things to say. The last two are
properties checked on every sample of a long run, and their size is a
consequence of the run length rather than a measure of anything.

---

## The rules the tests follow

**Nothing assigns a state.** Every test drives the real `semaphore_update()` at
a real tick and waits for the machine to arrive where it is going. A test that
sets the state it wants to examine has proved that the state can be assigned
to, which nobody doubted.

**The observer writes its own version.** `test_integration.c` carries its own
conflict tables, its own rule about what has to follow a green, its own
stopwatch on the yellow. An observer that consults the same table the
controller used cannot disagree with it. Same reason `test_monitor.c` writes
the four-way conflict matrix out again instead of importing
`plan_cross4.conflict`.

**Faults are injected downstream of the controller.** Every injection in
`test_monitor.c` and in `sem_sim --fault=` corrupts the *field readback*, not
the state machine. The controller behaves correctly throughout — which is
exactly the case the monitor exists for, and the only case where its
independence means anything.

**Timing is exact or it is stated.** The tick is 10 ms and every interval in
every plan is a multiple of it, so the interval tests use equality rather than
a tolerance. Where a tolerance is genuinely needed — a response measured from
an event that can land mid-tick — `CHECK_WITHIN` takes the slack as an
argument and the argument is one tick.

---

## What each one covers

### `test_plan`

Every rejection case is a single-character-scale mistake in a plan that is
otherwise the real one: a conflict entry made one-way, a stage that also
releases a third approach, an approach dropped out of the stage that served it,
a pedestrian clearance lengthened past the green that has to contain it. Each
asserts both that the plan is refused and that it is refused *for the stated
reason* — a check that fires for the wrong cause is not the check under test.

The last case is the property the node depends on at boot: a rejected plan
leaves the controller latched in flashing red, and stays there.

### `test_phases`

The startup flash, the interval lengths, which approaches share a stage. Then
actuation: a green extended to one headway past the last car, an early call
that cannot move the minimum, a continuous queue that runs into the maximum, a
yellow that cannot be extended by anything. Then the pedestrian timings, the
rest-in-green on the T junction and the button press that interrupts it, and
the two ways out of service — the orderly one that runs the yellow and the
all-red first, and the fault that drops the green on the spot.

### `test_monitor`

A correct cycle, by hand, so a failure anywhere else in the file means
something. Then each fault class, including both sides of each threshold: a
yellow 200 ms under the minimum trips, a yellow exactly at it does not.

Two boundary cases that were easy to get wrong and are now pinned: a flashing
head does not look like a field mismatch (the command and the field are
resolved from the same clock), and a commanded drop into flash does not look
like a skipped yellow.

Then the injections through a running unit, and half an hour of correct
operation under randomised traffic to prove the monitor stays quiet — an alarm
that eventually goes off on nothing is an alarm nobody will believe.

### `test_proto`

Encoding, decoding, round trips across the whole field range, a frame that does
not fit leaving the caller's buffer untouched, a decoder that rejects a frame
without `id` and `state` and tolerates one carrying keys it has never heard of.

Then the part with a claim attached. `ttc_ms` says the state cannot change
before then, and the only way to check that is to read the frame, run a copy of
the intersection forward, and compare against when the state actually changed.
On the fixed plan that has to be exact to the tick, on every sample across two
full cycles. On the actuated plan it has to be a bound that never overstates,
on every sample across a minute of randomised traffic — and the test asserts
that at least one sample really was inexact, because otherwise it proved
nothing about bounds.

### `test_integration`

Ten minutes per plan under randomised traffic, with an independent observer
checking on every tick:

- no two conflicting approaches occupying the intersection at once
- no pedestrian walking or clearing into a road that is running
- the broadcast state agreeing with the head
- green followed by yellow followed by red, never anything else
- yellow exactly the plan's yellow, every time
- the full all-red measured from the field between a release and a conflicting green
- every approach served, and none waiting longer than two cycles at the maximum green
- the monitor silent throughout

Four runs: the four-way busy, the four-way nearly empty, the fixed plan, and
the T junction — where the main road rests in green, so its green is allowed to
run past the stage maximum and the observer is told so explicitly rather than
having the check quietly weakened for everybody.

---

## Adding an intersection

Add the tables to `plans.c` and `intersection_plan_check()` will tell you what
is wrong with them. Then:

1. add it to `test_plan.c`'s `test_real_plans_pass()`
2. add a `run_plan()` line to `test_integration.c` with the conflict tables
   **written out again from the geometry**, not copied from `plans.c`
3. `make test`

The second step is the one worth insisting on. Copying the bit pattern across
makes the integration test agree with whatever `plans.c` says, including
whatever it says wrongly.

---

## The board layer

```bash
make node
```

compiles `firmware/semaphore_node/` against `tools/hal_stub/stm32f1xx_hal.h`, a
header that declares the handful of HAL names the node touches and defines none
of them. Code that is never compiled anywhere is code that is wrong in ways
nobody has found yet, and a misspelled pin does not need a board to catch.

The stub has already earned its place once: the real HAL's `GPIOC` is an
address constant, and a first version of the stub that made it a variable
turned the pin table in `lamp_gpio.c` into something that would not compile on
the target. That is exactly the class of thing it is there for.

A clean `make node` says the board layer compiles. It says nothing about
whether the hardware works.
