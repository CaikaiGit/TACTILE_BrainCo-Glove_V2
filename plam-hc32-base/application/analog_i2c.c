
#include "analog_i2c.h"

#include "time.h"
#include <stddef.h>

#define I2C_DELAY_US 5 // 基础延时(微秒)，根据I2C时钟频率调整

static void analog_i2c_write_sda(analog_i2c_t *analog_i2c, uint8_t level)
{
    if (analog_i2c->write_sda != NULL) {
        analog_i2c->write_sda(analog_i2c->sda_ctx, level);
    }
}

static void analog_i2c_write_scl(analog_i2c_t *analog_i2c, uint8_t level)
{
    if (analog_i2c->write_scl != NULL) {
        analog_i2c->write_scl(analog_i2c->scl_ctx, level);
    }
}

static uint8_t analog_i2c_read_sda_pin(analog_i2c_t *analog_i2c)
{
    if (analog_i2c->read_sda != NULL) {
        return analog_i2c->read_sda(analog_i2c->sda_ctx);
    }
    return 1U;
}

void i2c_sda_toggle_tx(analog_i2c_t *analog_i2c, uint8_t en)
{
    if (analog_i2c->set_sda_output != NULL) {
        analog_i2c->set_sda_output(analog_i2c->sda_ctx, en);
    }
}

void analog_i2c_start(analog_i2c_t *analog_i2c)
{

    i2c_sda_toggle_tx(analog_i2c, 1);

    analog_i2c_write_sda(analog_i2c, 1U);
    analog_i2c_write_scl(analog_i2c, 1U);

    delay_us(I2C_DELAY_US);

    analog_i2c_write_sda(analog_i2c, 0U);
    delay_us(I2C_DELAY_US);

    analog_i2c_write_scl(analog_i2c, 0U);
    delay_us(I2C_DELAY_US * 10);
}

void analog_i2c_stop(analog_i2c_t *analog_i2c)
{

    i2c_sda_toggle_tx(analog_i2c, 1);

    analog_i2c_write_sda(analog_i2c, 0U);
    analog_i2c_write_scl(analog_i2c, 1U);
    delay_us(I2C_DELAY_US);

    analog_i2c_write_sda(analog_i2c, 1U);
    delay_us(I2C_DELAY_US);
}

void analog_i2c_ask(analog_i2c_t *analog_i2c)
{
    i2c_sda_toggle_tx(analog_i2c, 1);

    analog_i2c_write_sda(analog_i2c, 0U);
    delay_us(I2C_DELAY_US);

    analog_i2c_write_scl(analog_i2c, 1U);
    delay_us(I2C_DELAY_US);

    analog_i2c_write_scl(analog_i2c, 0U);
    delay_us(I2C_DELAY_US);

    analog_i2c_write_sda(analog_i2c, 1U);
    delay_us(I2C_DELAY_US);
}

bool analog_i2c_wait_ask(analog_i2c_t *analog_i2c)
{
    uint32_t timeout = 0;

    analog_i2c_write_sda(analog_i2c, 1U);
    delay_us(I2C_DELAY_US);
    i2c_sda_toggle_tx(analog_i2c, 0);
    analog_i2c_write_scl(analog_i2c, 1U);
    delay_us(I2C_DELAY_US);

    while (analog_i2c_read_sda_pin(analog_i2c)) {
        timeout++;
        if (timeout > 2000) {
            analog_i2c_write_scl(analog_i2c, 0U);
            delay_us(I2C_DELAY_US);
            return false; // NACK
        }
    }

    analog_i2c_write_scl(analog_i2c, 0U);
    delay_us(I2C_DELAY_US);
    return true; // ACK
}

void analog_i2c_nack(analog_i2c_t *analog_i2c)
{
    i2c_sda_toggle_tx(analog_i2c, 1);

    analog_i2c_write_sda(analog_i2c, 1U);

    delay_us(I2C_DELAY_US);

    analog_i2c_write_scl(analog_i2c, 1U);

    delay_us(I2C_DELAY_US);

    analog_i2c_write_scl(analog_i2c, 0U);
    delay_us(I2C_DELAY_US);
}

bool analog_i2c_send_data(analog_i2c_t *analog_i2c, uint8_t data)
{
    i2c_sda_toggle_tx(analog_i2c, 1);

    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            analog_i2c_write_sda(analog_i2c, 1U);
        }
        else {
            analog_i2c_write_sda(analog_i2c, 0U);
        }

        delay_us(I2C_DELAY_US);

        analog_i2c_write_scl(analog_i2c, 1U);
        delay_us(I2C_DELAY_US);

        analog_i2c_write_scl(analog_i2c, 0U);
        data <<= 1;
        delay_us(I2C_DELAY_US);
    }

    if (!analog_i2c_wait_ask(analog_i2c)) {
        return false;
    }
    return true;
}

void analog_i2c_read_data(analog_i2c_t *analog_i2c, uint8_t *data, bool last)
{
    i2c_sda_toggle_tx(analog_i2c, 0);
    *data = 0;
    for (uint8_t i = 0; i < 8; i++) {
        *data <<= 1;
        analog_i2c_write_scl(analog_i2c, 1U);
        delay_us(I2C_DELAY_US);

        if (analog_i2c_read_sda_pin(analog_i2c)) {
            *data |= 0x01;
        }
        analog_i2c_write_scl(analog_i2c, 0U);
        delay_us(I2C_DELAY_US);
    }
    if (last) {
        analog_i2c_nack(analog_i2c);
    }
    else {
        analog_i2c_ask(analog_i2c);
    }
}
