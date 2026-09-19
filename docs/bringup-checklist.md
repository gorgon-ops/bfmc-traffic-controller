# Bring-up

The order to find out whether the board works in. Nothing here has been done
yet — the unit is designed and type-checked and has never been powered.

The principle throughout: **never let the first thing a head does be a green.**
Every step below either leaves the intersection in flashing red or checks one
output in isolation, and the plan does not run until the wiring has been proved
by hand.

---

## 1. Before power

- [ ] Continuity from every ULN2803 output to the LED it is supposed to drive.
      Twelve vehicle pins, four pedestrian. Check them against the table in
      `board_config.h`, not against memory.
- [ ] Series resistor on every LED. A ULN2803 will happily destroy all sixteen.
- [ ] The lamp supply switch on PB12 is wired so that **low means off**. If it
      is inverted, a fault trip turns every head on.
- [ ] ESP module powered from its own regulator, not from the NUCLEO's 3V3 pin.
      An ESP transmit peak will brown out the ST-LINK's regulator and reset the
      processor, and a processor that resets mid-cycle is the failure this
      whole project is about.
- [ ] Common ground between the STM32, the ESP and the head board.

## 2. Outputs, one at a time

Before loading the real firmware. A short loop that walks one pin at a time.

- [ ] Each of the sixteen lamps lights, alone, in the order of the table. An
      approach that lights in the wrong bulb order is a connector on backwards
      and is much easier to find now than inside a running cycle.
- [ ] With each pin driven, `HAL_GPIO_ReadPin()` on that same pin reads back
      what was written. This is the readback the conflict monitor depends on.
      If a pin reads stuck, the monitor will trip on it forever and it is
      better to know why.
- [ ] PB12 low: every head goes dark. PB12 high: they come back.

## 3. Inputs

- [ ] Each detector reads idle with nothing in front of it and asserted with a
      car on the line. Check the *asserted* sense is low.
- [ ] Unplug a detector. It must read **idle**, not asserted. A detector that
      fails asserted holds its green to the maximum every cycle forever.
- [ ] Both crossing buttons, and the service flash switch.
- [ ] Watch the debounce: `BOARD_DEBOUNCE_MS` is 40 ms, four ticks. A button
      that produces two calls per press needs more, not a change in the plan.

## 4. First run of the real firmware

- [ ] Power up and watch the first three seconds. Every head must flash red for
      `INT_STARTUP_FLASH_MS`, then all show steady red for the all-red, and
      only then may anything go green. If a green appears inside the first
      three seconds, stop and find out why before anything else.
- [ ] Debug UART at 115200 shows the state, the stage and the cycle count.
- [ ] `tick Nus` on that line is the worst tick so far, against a 10,000 µs
      budget. Anything within an order of magnitude of the budget needs
      explaining before the unit goes on a track.
- [ ] Let it run a full cycle with nothing on the detectors. Time the yellow
      with a stopwatch and compare it against `INT_YELLOW_MS`. If the tick
      timer is misconfigured this is where it shows, and every interval in the
      plan is wrong by the same ratio.

## 5. The monitor, deliberately

The point of a conflict monitor is that you get to test it.

- [ ] With the unit running, short one head's green input to its enable — or
      more simply, jumper a conflicting green on the head board. The unit must
      go to flashing red **on the spot**, drop PB12, and print the fault, the
      two group numbers and the timestamp.
- [ ] It must stay there. Remove the jumper; it stays in flash. Only a reset
      brings it back, and the reset must come back through the full startup
      flash rather than straight into a green.
- [ ] Unplug a lamp from a head that is commanded on and confirm what happens:
      with the pin readback alone, **nothing** — this is the known hole in
      `docs/conflict.md` and it is worth seeing it not work, so that nobody
      later assumes it does.

## 6. The broadcast

- [ ] `python tools/listen.py --port 5007` on a laptop on the same network.
      Four ids, ten frames a second each.
- [ ] Compare the `state` in the frames against the heads, by eye, through a
      full cycle. Particularly the pre-green: the head shows red and yellow,
      the frame must say `"state":0`.
- [ ] Trip the monitor again and confirm `"fault":1` appears and the countdown
      goes to zero and inexact.
- [ ] Pull power from the ESP module. The lamps must carry on exactly as
      before. The broadcast is a convenience; the intersection is not.

## 7. On the track

- [ ] All four units up, and check the flash phases are not synchronised in a
      way that looks like a pattern. They are each free-running off their own
      tick, which is what `docs/` says they are — there is no coordination and
      no green wave.
- [ ] One full session with the marshals knowing which switch is the service
      flash.
