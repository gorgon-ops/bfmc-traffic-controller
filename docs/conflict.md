# The conflict monitor

What it checks, what it cannot check, and how far the independence actually
goes on this hardware.

---

## Why there are two of them

A real signal cabinet contains two computers. One runs the phase plan. The
other — the conflict monitor, or in American practice the malfunction
management unit — watches the voltage actually present on the lamp wires and
cuts the field to flashing red when what it sees is unsafe, regardless of what
the first computer believes it commanded.

They are separate because the failures that matter are the ones the controller
cannot see:

- an output driver shorted on
- a connector on the wrong head
- a plan loaded with the conflict matrix of a different junction
- firmware that has stopped running while its outputs stay latched

In every one of those the state machine is behaving perfectly. It has a
consistent internal state, it is advancing through its stages on time, and the
intersection is showing something else.

`firmware/common/app/conflict_monitor.c` is that second computer.

---

## What it checks

Each tick it is handed the commanded aspects and the bulb vector measured on
the heads, and it judges the second one.

| check | what it catches |
|---|---|
| **conflict** | two groups permissive or clearing at once that its matrix forbids |
| **short yellow** | a yellow that ended before `MON_MIN_YELLOW_MS` |
| **skipped yellow** | green straight to red with no clearance at all |
| **short clearance** | a green that arrived less than `MON_MIN_ALL_RED_MS` after a conflicting group stopped occupying the intersection |
| **bad aspect** | a bulb combination that is not an aspect — red and green lit together, say |
| **field mismatch** | the bulbs have not matched the commanded aspect for longer than the settling window |
| **stall** | nobody has shown it anything for `MON_MAX_GAP_MS` |

Three details that are easy to get wrong:

**Yellow counts as occupancy.** A car on a yellow is still inside the box, so a
green on a conflicting approach while another shows yellow is a conflict, not a
near miss. The monitor treats permissive and clearing identically.

**It has its own numbers.** `MON_MIN_YELLOW_MS` and `MON_MIN_ALL_RED_MS` are
not the values the plan runs. They are floors that sit below them, so that a
plan edited down to something unsafe trips its own monitor rather than quietly
running. `intersection_plan_check()` refuses a plan that starts below them, so
the two failures do not have to be diagnosed together.

**It has its own conflict matrix.** `conflict_monitor_init()` copies it. Two
units reading one array cannot disagree about what it says, and the monitor
exists to be able to disagree.

### The two deliberate exceptions

*The settling window.* The field is read at the start of a tick and written at
the end of it, so the readback always trails the command by one tick and on
every single transition the two legitimately disagree for that tick. A
mismatch has to persist for `MON_FIELD_GRACE_MS` (100 ms, against a 1000 ms
all-red) before it counts. Without it the monitor would trip on correct
operation, which is worse than not having one.

*The drop into flash.* Green to flashing red with no yellow is exactly the
skipped-yellow pattern, done on purpose: the intersection is being taken out of
service because something is already wrong, and sitting through a yellow first
would hold a known-bad indication up for another second and a half. The monitor
recognises it from the commanded aspect. This is the one place it consults the
command for anything other than comparing it against the field, and it is
worth noticing as a small hole in the independence.

An *orderly* flash — the maintenance switch — does not use the exception. It
runs the yellow and the all-red first and arrives at flash from an
intersection that is already entirely red.

---

## The fault latches

Once tripped, every subsequent call returns the same fault without
re-examining anything, and `intersection_reset()` is the only way out.

An intersection that recovered from a conflict on its own would show you the
conflict again, on a schedule, and never tell you it had happened. Latching is
what turns an intermittent short into a unit sitting in flashing red with a
timestamp and a group number on the debug UART.

The first fault is also the only one recorded. A conflict produces a cascade of
consequences — the clearance is wrong too, the yellow after it is wrong too —
and the first cause is the one worth keeping.

---

## What it cannot check, on this board

This is the part that a safety claim has to say out loud.

**The monitor shares a core with the controller.** In a cabinet the monitor is
a separate board on separate power that can cut the field supply without the
controller's cooperation. Here, a controller that has hard-faulted with a green
latched on its output pins takes the monitor down with it, and nothing runs to
notice. `BOARD_LAMP_SUPPLY` (PB12) drops the supply feeding every head on a
trip, which covers a stuck *driver*, but dropping it is still an action this
firmware has to be alive to take.

Closing that gap properly means a second microcontroller — an ATtiny is
plenty — holding the conflict matrix, reading the same lamp lines, and driving
the supply relay through an AND with the main processor's watchdog output.
That is a board revision, not a code change, and it is the single most
worthwhile thing on this project's list.

**The readback measures voltage at the processor, not current in the lamp.**
`lamp_gpio_read()` reads the input register of pins configured as outputs, so
it reports what is actually on the pad. That catches a pin held down by a
shorted driver, a pin pulled up by a miswired head, and a port whose clock
never got enabled.

It does not catch a LED that has failed open. The pin reads exactly right and
nothing is lit. A dark red head is a head a car will read as *no signal here*,
which on this track means an unsignalised junction and a car proceeding with
caution — survivable, but not what the intersection thinks it is showing.
`docs/hardware.md` has the sense-resistor version that would catch it.

**The host tests pass `NULL` for the readback where they are not injecting.**
That makes the field comparison compare the command against itself, which is
vacuous. Every other check still works on those runs;
`tests/test_monitor.c` drives the field comparison explicitly with vectors
that disagree, and `tests/test_node.c` goes further by jamming an actual pin
in the fake HAL, so the readback the monitor sees is one the firmware tried
and failed to change.

**The monitor's conflict matrix is copied from the plan at init.** That gives
it independence from later corruption of one copy, and no independence at all
from a plan that was wrong to begin with - both copies are then equally wrong.
On real hardware the second copy should be loaded separately and verified
separately. What does help here is `make prove`, which model-checks
`intersection_plan_check()` over an arbitrary matrix rather than over the
matrices that happen to be in `plans.c`.

---

## Seeing it work

```bash
make faults
```

runs each class through a correct controller with the fault injected into the
field readback — downstream of the state machine, so the controller is
behaving correctly throughout, which is the case the monitor exists for.

```
injecting: a shorted driver puts a green on a conflicting approach

   10.01s  monitor: conflicting greens (groups 0 and 1)
   10.01s  intersection: flash, heads now *R *R *R *R
   10.01s  broadcast: {"id":1,"state":0,"ttc_ms":0,"exact":0,"fault":1}
   12.01s  still flash - the latch needs a deliberate reset
```

The heads go safe on the same tick the conflict is detected, not the next one:
`semaphore_update()` re-resolves the outputs after the monitor has had its say,
which is why it returns the bulb vectors rather than writing them as it goes.
