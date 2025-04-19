/**
 * @file tm1637.h
 * @brief Driver for TM1637 LED display on ESP‑IDF.
 *
 * This is a tiny bit‑banged driver that talks to the TM1637 controller
 * using two GPIOs (CLK and DIO).  It is intentionally minimal and does
 * not allocate any RTOS resources.  All timings follow the datasheet.
 */
#ifndef TM1637_H
#define TM1637_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <driver/gpio.h>
#include <rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* ------------------------------------------------------------------------
 * Compile‑time configuration
 * --------------------------------------------------------------------- */

/** µs delay inserted between pin toggles. 1 µs is plenty at 80 MHz. */
#ifndef TM1637_DATA_DELAY_US
#define TM1637_DATA_DELAY_US   1U
#endif

/* Command bytes as specified by the TM1637 datasheet */
#define TM1637_I2C_COMM1       0x44  /**< Command 1: set data & addr auto‑inc */
#define TM1637_I2C_COMM2       0xC0  /**< Command 2: write display data       */
#define TM1637_I2C_COMM3       0x88  /**< Command 3: display control + ON     */

/* ------------------------------------------------------------------------
 * Type definitions
 * --------------------------------------------------------------------- */

/**
 * @brief Handle describing a TM1637 display instance.
 *
 * One handle corresponds to exactly one TM1637 chip.
 */
typedef struct tm1637__led_driver {
    uint8_t CLK_PIN;     /**< GPIO connected to CLK. */
    uint8_t DIO_PIN;     /**< GPIO connected to DIO. */
    uint8_t brightness;  /**< Brightness level 0–7.  */
} tm1637__led_driver_t;

/* Backwards‑compatibility alias */
typedef tm1637__led_driver_t tm1637_t;

/* ------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

tm1637__led_driver_t *tm1637_init(uint8_t clk_pin, uint8_t dio_pin);
void tm1637_set_brightness(tm1637__led_driver_t *drv, uint8_t level);
void tm1637_set_seg_raw(tm1637__led_driver_t *drv, uint8_t segment_idx, uint8_t data);
void tm1637_turnoff(tm1637__led_driver_t *drv);
void tm1637_turnon(tm1637__led_driver_t *drv);
void tm1637_find_leds(tm1637__led_driver_t *drv, int delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* TM1637_H */
