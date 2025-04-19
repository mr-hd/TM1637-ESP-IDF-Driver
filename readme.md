# TM1637 ESP‑IDF Driver

Tiny, dependency‑free driver for **TM1637** 7‑segment LED display modules,
implemented in pure C for **Espressif ESP‑IDF**.

## Features

* Bit‑banged two‑wire protocol (CLK & DIO) – no I²C / peripheral required.
* Adjustable global brightness (0–7).
* Helper to visualise segment mapping.
* Fully FreeRTOS‑safe: no heap allocations after `tm1637_init()`, no blocking
  longer than 1 µs per bit, and no interrupts disabled.

## Getting started

### Wiring

| TM1637 Pin | ESP32 GPIO |
|------------|------------|
| VCC        | 3V3        |
| GND        | GND        |
| DIO        | Any GPIO   |
| CLK        | Any GPIO   |

### Basic usage

```c
#include "tm1637.h"

tm1637__led_driver_t *display = tm1637_init(9, 8);
tm1637_set_brightness(display, 7);        // max‑level brightness

// Turn on zero segment 2nd LED
tm1637_set_seg_raw(display, 0x00, 0x01);

```

See `tm1637_find_leds()` for a mapping helper.
```c
tm1637_find_leds(display, 1000);	// 1000ms delay for segment determination
```


