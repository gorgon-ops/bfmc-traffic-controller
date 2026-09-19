# Testing

```bash
make            # everything: seven test programs, the simulation, the board layer
make test       # just the test programs
```

925,389 assertions, no hardware, about thirty-six seconds.

| program | assertions | what it is for |
|---|---|---|
| `test_plan` | 41 | the configuration check, driven with plans that are wrong on purpose |
| `test_phases` | 127 | the aspect-to-bulb mapping, then the sequence and the length of every interval in it |
| `test_monitor` | 869 | the conflict monitor, against field output no controller would produce |
| `test_proto` | 20,223 | the frame, and whether its countdown is ever early |
| `test_node` | 112 | the board layer on a fake HAL: debounce, tick order, the fault path |
| `test_integration` | 904,017 | forty simulated minutes, every invariant checked on every tick |
| `fuzz_proto` | 27 seeds + 200k mutations | the parser, against an independently written oracle |
| `cbmc_harness` | build check | that the proof harnesses still compile and still match the code |

The counts are lopsided on purpose. The hand-written files have as many cases
as there are things to say. `test_integration` and `fuzz_proto` check a
property on every sample of a long run, and their size is a consequence of the
run length rather than a measure of anything.

And four more that need tooling this repository does not vendor. Each prints
what is missing rather than failing obscurely:

```bash
make coverage   # gcov over the portable core          (gcov ships with gcc)
make fuzz       # libFuzzer + ASan + UBSan             (needs clang)
make prove      # CBMC over the conflict invariant     (needs cbmc)
make arm        # cross-compile and report footprint   (needs arm-none-eabi-gcc)
make test SAN=1 # the whole suite under ASan and UBSan (needs a sanitizer runtime)
```

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
`plan_cross4.conflict`, and the same reason `fuzz_proto.c` carries a second
parser.

**Faults are injected downstream of the controller.** Every injection in
`test_monitor.c`, in `test_node.c` and in `sem_sim --fault=` corrupts the
*field readback* or the pin itself, never the state machine. The controller
behaves correctly throughout — which is exactly the case the monitor exists
for, and the only case where its independence means anything.

**Timing is exact or it is stated.** The tick is 10 ms and every interval in
every plan is a multiple of it, so the interval tests use equality rather than
a tolerance. Where a tolerance is genuinely needed — a response measured from
an event that can land mid-tick — `CHECK_WITHIN` takes the slack as an
argument and the argument is one tick.

**A test that cannot fail is not a test.** Every mechanism here has been run
against a broken version to confirm it notices; where that check is not
obvious, it is written down beside the test. Three of them are worth naming:

- the differential fuzzer reports 2,002 violations against the pre-fix
  decoder and 0 against the current one
- the delay regression guard reported `+1713% WORSE` when a bad edit corrupted
  the arrival rates, which is how its detection path got exercised
- the CBMC harnesses' concrete `main()` caught the harnesses driving
  uninitialised memory, within a second of first being run

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

First the translation everything else is built on: every aspect to its bulb
vector, which bulb combinations are legal aspects at all, and which of them
count as permissive or clearing. The one worth being sure about is that red
and yellow together is neither.

Then the startup flash, the interval lengths, which approaches share a stage,
and actuation: a green extended to one headway past the last car, an early
call that cannot move the minimum, a continuous queue that runs into the
maximum, a yellow that cannot be extended by anything. Then the pedestrian
timings, the rest-in-green on the T junction and the button press that
interrupts it, and the two ways out of service.

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

Plus the overflow family, which is a regression suite: `"id":4294967297` used
to decode as id 1, and `"state":4294967298` used to decode as a **green**,
because the accumulator wrapped and every range check then ran against the
wrapped value. Rejecting the digits is the fix; these cases are what stops it
coming back.

Then the part with a claim attached. `ttc_ms` says the state cannot change
before then, and the only way to check that is to read the frame, run a copy of
the intersection forward, and compare against when the state actually changed.
On the fixed plan that has to be exact to the tick, on every sample across two
full cycles. On the actuated plan it has to be a bound that never overstates,
on every sample across a minute of randomised traffic — and the test asserts
that at least one sample really was inexact, because otherwise it proved
nothing about bounds.

### `test_node`

The board layer, on a fake HAL that gives pins a level, a tick counter a test
can advance, and a UART that keeps what was written to it. Until this existed,
the debounce, the order of operations inside a tick, the fault path and the
poll scheduling were the only code in the project nobody had ever run.

- the pins the board writes are the pins the plan asked for, checked against
  the map in `board_config.h` written out again
- a detector pulse shorter than the debounce window is not a car; a held one
  is; a line that *rattles* never settles however long it rattles for
- a button is an edge, so holding it down asks once
- a jammed output pin ends with the unit faulted, the pattern flashing **and**
  the lamp supply relay dropped
- a plan with more heads than the board is wired for is refused at boot
- two nodes in one process do not interfere — the test that could not be
  written while the poll path kept its state in function-level statics

What it still does not prove: that the pin numbers in `board_config.h` are the
pins the hardware is wired to. A fake HAL will happily agree with a pin map
that is wrong in exactly the same way the firmware is.

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

### `fuzz_proto`

`sem_proto_decode()` is the only parser of untrusted input in the project:
everything else consumes structures the firmware built itself. It is
hand-rolled, and it had an integer overflow in it.

The properties are self-consistency — a decoded frame is in range, re-encodes,
and round-trips to itself — plus a **differential check against a second,
independently written reader** that does its arithmetic in 64 bits. That last
one is the only property that catches the bug class this file exists for, and
finding that out was worth more than the other three: a wrapped `id` produces
a frame that is entirely well-formed. Nothing about the output is wrong. What
is wrong is that the output exists at all, for an input that should have been
refused, and no property that only inspects the result can see it.

The same file is both a libFuzzer target (`make fuzz`) and a replay driver
that needs no clang (`make test`), so the corpus is a regression suite
everywhere even though coverage-guided fuzzing only runs in CI.

### `cbmc_harness`

Not a test. Three harnesses for bounded model checking — no conflicting green,
the aspect sequence, and `intersection_plan_check()` over an arbitrary plan
rather than the ones in `plans.c`. `make prove` runs them under CBMC, which
answers for *all* call sequences to the unwind depth rather than for the ones a
random generator happened to produce.

`make test` builds them with a normal compiler and executes them concretely.
That proves nothing about the properties and everything about the harnesses
still matching the code they reason about — a harness that no longer compiles
is a harness nobody notices has stopped being run.

---

## Coverage

```bash
make coverage
```

```
  lamps.c                 95.45% of 44
  intersection.c          92.11% of 304
  conflict_monitor.c      87.88% of 99
  sem_proto.c             93.44% of 122
  semaphore.c             96.10% of 77
  TOTAL                   92.41% of 646
```

`plans.c` does not appear: it is a table of static data with no executable
lines, which gcov correctly declines to have an opinion about.

The number moved once already for a real reason. `lamps.c` sat at 54% because
the aspect-to-bulb mapping was only ever exercised through whichever aspects a
running plan happened to produce — the flashing yellow, the dark head and the
diagnostic names were never reached. Those are now tested directly, which is
the point of measuring: the uncovered lines were exactly the ones nobody had
thought about.

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

## Guarding the numbers

```bash
make check
```

The delay figures are the project's headline claim and nothing was stopping a
refactor from quietly costing twenty percent of them — the test suite checks
that the controller is correct, and correctness does not notice a green that
got shorter. `make check` reruns five cases against recorded baselines and
fails outside 8% in **either** direction: a delay that improved by a quarter
without anyone trying usually means an interval got shorter, and the intervals
are the safety argument.

Re-measure with `sem_sim --baseline`, which prints the table ready to paste.
A baseline updated without an explanation in the commit message is a
regression with a commit on top of it.

---

## The board layer

```bash
make node
```

compiles `firmware/semaphore_node/` against `tools/hal_stub/stm32f1xx_hal.h`, a
header that declares the handful of HAL names the node touches and defines none
of them.

The stub has earned its place once already: the real HAL's `GPIOC` is an
address constant, and a first version of the stub that made it a variable
turned the pin table in `lamp_gpio.c` into something that would not compile on
the target.

A clean `make node` says the board layer compiles. `test_node` says it
behaves. Neither says anything about whether the hardware works — and
`make arm` says only that it cross-compiles, which is still more than a host
build says, because host `int` widths and host ABI are not the target's.
