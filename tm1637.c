/**
 * @file tm1637.c
 * @brief Tiny bit‑banged driver for the TM1637 LED driver IC.
 *
 * The implementation follows the timing diagram in the TM1637 datasheet.
 * All GPIO bit‑banging happens with interrupts enabled; the chip tolerates
 * large timing margins so the ESP‑IDF scheduler does not interfere.
 */

#include "tm1637.h"

static const char *TAG = "tm1637";

/* ------------------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------------*/

/* Convenience macro – both code and readability benefit. */
#define DELAY() ets_delay_us(TM1637_DATA_DELAY_US)

/**
 * @brief Generate the start condition (CLK high, DIO high → low).
 */
static void tm1637__data_start(tm1637__led_driver_t *drv)
{
    gpio_set_level(drv->CLK_PIN, 1);
    DELAY();
    gpio_set_level(drv->DIO_PIN, 1);
    DELAY();
    gpio_set_level(drv->DIO_PIN, 0);
    DELAY();
    gpio_set_level(drv->CLK_PIN, 0);
    DELAY();
}

/**
 * @brief Generate the stop condition (CLK low → high while DIO high).
 */
static void tm1637__data_stop(tm1637__led_driver_t *drv)
{
    gpio_set_level(drv->CLK_PIN, 0);
    DELAY();
    gpio_set_level(drv->DIO_PIN, 0);
    DELAY();
    gpio_set_level(drv->CLK_PIN, 1);
    DELAY();
    gpio_set_level(drv->DIO_PIN, 1);
    DELAY();
}

/**
 * @brief Write one byte and read the ACK bit.
 *
 * The function respects the LSB‑first convention required by TM1637.
 */
static void tm1637__send_byte(tm1637__led_driver_t *drv, uint8_t byte)
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        gpio_set_level(drv->CLK_PIN, 0);
        DELAY();

        /* Put data bit on the bus. */
        gpio_set_level(drv->DIO_PIN, (byte & 0x01));
        DELAY();

        /* Rising edge latches the bit into TM1637. */
        gpio_set_level(drv->CLK_PIN, 1);
        DELAY();

        byte >>= 1;
    }

    /* Read acknowledgement bit. */
    gpio_set_level(drv->CLK_PIN, 0);
    DELAY();
    gpio_set_level(drv->DIO_PIN, 1);              /* Release line      */
    DELAY();
    gpio_set_level(drv->CLK_PIN, 1);
    DELAY();

    /* Temporarily switch DIO to input to sample ACK. */
    gpio_set_direction(drv->DIO_PIN, GPIO_MODE_INPUT);
    uint8_t ack = gpio_get_level(drv->DIO_PIN);
    gpio_set_direction(drv->DIO_PIN, GPIO_MODE_OUTPUT);

    /* Active‑low ACK – pull DIO low if slave responded. */
    if (ack == 0)
    {
        gpio_set_level(drv->DIO_PIN, 0);
        DELAY();
    }

    /* Prepare for next byte */
    gpio_set_level(drv->CLK_PIN, 0);
    DELAY();
}

/* ------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------*/

tm1637__led_driver_t *tm1637_init(uint8_t pin_clk, uint8_t pin_data)
{
    tm1637__led_driver_t *drv = calloc(1, sizeof(tm1637__led_driver_t));
    if (!drv) {
        ESP_LOGE(TAG, "Out of memory");
        return NULL;
    }

    drv->CLK_PIN    = pin_clk;
    drv->DIO_PIN    = pin_data;
    drv->brightness = 0x07;           /* Default: max brightness */

    /* Configure GPIOs */
    gpio_set_direction(pin_clk,  GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_data, GPIO_MODE_OUTPUT);
    gpio_set_level(pin_clk,  0);
    gpio_set_level(pin_data, 0);

    return drv;
}

void tm1637_set_brightness(tm1637__led_driver_t *drv, uint8_t level)
{
    drv->brightness = (level > 7U) ? 7U : level;
}

void tm1637_set_seg_raw(tm1637__led_driver_t *drv, uint8_t seg_num, uint8_t data)
{
    /* 1) Address write cmd – automatic address increment disabled */
    tm1637__data_start(drv);
    tm1637__send_byte(drv, TM1637_I2C_COMM1);
    tm1637__data_stop(drv);

    /* 2) Write data byte to selected address */
    tm1637__data_start(drv);
    tm1637__send_byte(drv, (seg_num | TM1637_I2C_COMM2));
    tm1637__send_byte(drv, data);
    tm1637__data_stop(drv);

    /* 3) Update brightness and enable display */
    tm1637__data_start(drv);
    tm1637__send_byte(drv, (drv->brightness | TM1637_I2C_COMM3));
    tm1637__data_stop(drv);
}

void tm1637_turnoff(tm1637__led_driver_t *drv)
{
    for (uint8_t i = 0; i < 8; ++i) {
        tm1637_set_seg_raw(drv, i, 0x00);
    }
}

void tm1637_turnon(tm1637__led_driver_t *drv)
{
    for (uint8_t i = 0; i < 8; ++i) {
        tm1637_set_seg_raw(drv, i, 0xFF);
    }
}

void tm1637_find_leds(tm1637__led_driver_t *drv, int delay_ms)
{
    for (uint8_t pos = 0; pos < 8; ++pos) {
        for (uint8_t seg = 0; seg < 8; ++seg) {
            ESP_LOGI(TAG, "Pos %u seg %u", pos, seg);
            tm1637_set_seg_raw(drv, pos, 1U << seg);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}
