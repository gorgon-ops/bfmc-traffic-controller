# The hardware

One intersection unit. Four vehicle heads, two pedestrian heads, four stop-line
detectors, and a Wi-Fi module that does nothing but put lines onto the network.

**Status: designed, tested on the host, not yet built.** `make node` compiles
the board layer against a stubbed HAL and `make test` runs it against a fake
one - the debounce, the tick ordering, the fault path and the poll scheduling
all execute at the real tick rate. `make arm` cross-compiles it for the part
and reports the footprint.

None of that says the hardware works, and one gap is worth naming precisely:
a fake HAL agrees with a pin map that is wrong in exactly the same way the
firmware is. Only step 2 of `docs/bringup-checklist.md` settles the wiring.

---

## Bill of materials

| part | why this one |
|---|---|
| NUCLEO-F103RB | 72 MHz, 37 free GPIO on the Morpho headers, ST-LINK on board. A semaphore has no control loop and no deadline tighter than a tick; this is chosen for pin count and price, and four of them are affordable |
| ULN2803A ×2 | eight Darlington sinks each. Sixteen lamp outputs from two parts, with the flyback diodes already inside |
| ESP-01S or ESP32 devkit | the network. Talks to the STM32 over one UART |
| IRLZ44N or a high-side load switch | the lamp supply cutoff on PB12 |
| TCRT5000 ×4 | reflective stop-line detectors. An IR barrier across the lane works as well and is less sensitive to the colour of the car |
| 5 mm LEDs ×16, 330R ×16 | the heads themselves |

The scaled heads are LEDs, not lamps, so everything downstream of the ULN2803
is a few tens of milliamps and the "field" is a ribbon cable. The vocabulary in
the code is the full-size one anyway — heads, aspects, field output — because
it is the vocabulary the behaviour is copied from.

---

## Pin map

The authority is `firmware/semaphore_node/Core/Inc/board_config.h`; this is the
same table with the wiring notes attached.

### Vehicle heads

| group | approach | broadcast id | red | yellow | green |
|---|---|---|---|---|---|
| 0 | north | 1 | PC0 | PC1 | PC2 |
| 1 | east | 2 | PC3 | PC4 | PC5 |
| 2 | south | 3 | PC6 | PC7 | PC8 |
| 3 | west | 4 | PC9 | PC10 | PC11 |

Twelve consecutive pins on one port, in bulb order, deliberately. It makes the
table in `lamp_gpio.c` readable and it makes a miswired head obvious on the
bench: a whole approach shifts by one and shows yellow when it should show red.

### Pedestrian heads

| crossing | red | green |
|---|---|---|
| 0 | PB0 | PB1 |
| 1 | PB2 | PB10 |

No yellow. The flashing clearance is the red bulb blinking, at 2 Hz — twice the
vehicle flash rate, so a pedestrian head in clearance cannot be mistaken for a
vehicle head out of service seen out of the corner of an eye.

### Inputs — all active low, all with the internal pull-up

| function | pin |
|---|---|
| stop-line detector, groups 0–3 | PA0, PA1, PA4, PA5 |
| crossing button 0, 1 | PB4, PB5 |
| service flash switch | PC12 |

Active low is the safe direction. A detector whose cable falls off reads idle
and stops calling, and the plan already survives a detector that never calls —
`next_stage()` falls back to fixed-time round-robin rather than letting an
approach depend on a sensor that has died. A detector that failed the other way
would call continuously and hold its green to the maximum every cycle.

### Everything else

| function | pin | notes |
|---|---|---|
| lamp supply enable | PB12 | high to enable. Dropped on a monitor trip |
| ESP TX / RX | PA9 / PA10 | USART1, 115200 8N1 |
| debug TX | PA2 | USART2, the ST-LINK virtual COM |

TIM2 is the 100 Hz tick and drives nothing external.

---

## The readback, and the hole in it

Every lamp pin is configured as an output and read back with
`HAL_GPIO_ReadPin()`, which on an STM32 reads the input register — the voltage
actually on the pad, not the value written to it. The conflict monitor compares
that against what was commanded.

That catches a pin held down by a shorted driver, a pin pulled up by a miswired
head, and a port whose clock never got enabled.

It does not catch a LED that has failed open. The pin reads exactly right with
nothing lit.

The version that would catch it: put the ULN2803 commons through a 1R sense
resistor to ground, one per head, and take the top of each into an ADC channel.
A head with current flowing is showing something; a head commanded on with no
current is a dead bulb. Six ADC channels, six resistors, and
`lamp_gpio_read()` gains a second source it can AND with the pin state. It is
not built, and `docs/conflict.md` lists it as the second most worthwhile thing
on this project after making the monitor a separate part.

---

## The Wi-Fi module

The STM32 does not have a network stack and should not grow one. It writes one
frame per line to the module over USART1; the module puts each line into a UDP
datagram and broadcasts it. That is the whole protocol between them.

The ESP side, complete:

```cpp
// ESP8266 or ESP32. Nothing here decides anything.
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
const int PORT = 5007;

void setup() {
  Serial.begin(115200);
  WiFi.begin("<ssid>", "<password>");
  while (WiFi.status() != WL_CONNECTED) delay(200);
  udp.begin(PORT);
}

void loop() {
  static char line[128];
  static size_t n = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (n > 0) {
        IPAddress bcast = WiFi.localIP();
        bcast[3] = 255;
        udp.beginPacket(bcast, PORT);
        udp.write((const uint8_t *)line, n);
        udp.endPacket();
      }
      n = 0;
    } else if (n < sizeof(line)) {
      line[n++] = c;
    } else {
      n = 0;              // overlong line, drop it rather than split it
    }
  }
}
```

The split is not only convenience. Everything that decides what the lamps show
runs on the other side of that UART, on a processor with no DHCP client, no
retransmit timers and no radio firmware on it. A Wi-Fi stack that wedges takes
the broadcast down and leaves the intersection running — which is the right way
round. A car that stops seeing frames still has the lamps, and the lamps are
what the rules are written about.

---

## Timing

| | |
|---|---|
| tick | 100 Hz, TIM2 |
| tick budget | 10 ms |
| measured worst tick | not yet measured on silicon. `semaphore_node_worst_us()` reports it on the debug UART; the host run reports zero because the fake HAL's cycle counter does not advance, which is honest rather than useful |
| broadcast | 10 Hz per head, from the main loop |
| telemetry | 2 Hz, main loop |

NVIC: TIM2 above both USARTs. A tick delayed behind a telemetry transfer is a
tick whose `dt` is a lie, and every interval in the plan is counted in ticks.

The broadcast encoding happens in the main loop, not the tick. What the tick
publishes is `sem_frame_t` — five integers per head — and the main loop copies
all four under a short critical section before formatting them. Copying them
field by field would let a tick land in the middle and produce a frame whose
state came from before a change and whose countdown came from after it.
