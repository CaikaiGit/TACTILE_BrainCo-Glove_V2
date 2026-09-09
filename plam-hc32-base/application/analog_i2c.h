#ifndef __ANALOG_I2C_H
#define __ANALOG_I2C_H
#include "stdbool.h"
#include <stdint.h>

typedef void(AnalogI2cWritePinCallback)(void *ctx, uint8_t level);
typedef uint8_t(AnalogI2cReadPinCallback)(void *ctx);
typedef void(AnalogI2cSdaModeCallback)(void *ctx, uint8_t output_enable);

typedef struct och1970
{
    void *sda_ctx;
    void *scl_ctx;
    AnalogI2cWritePinCallback *write_sda;
    AnalogI2cWritePinCallback *write_scl;
    AnalogI2cReadPinCallback *read_sda;
    AnalogI2cSdaModeCallback *set_sda_output;
} analog_i2c_t;

void analog_i2c_start(analog_i2c_t *analog_i2c);
void analog_i2c_stop(analog_i2c_t *analog_i2c);
void analog_i2c_ask(analog_i2c_t *analog_i2c);
bool analog_i2c_wait_ask(analog_i2c_t *analog_i2c);
void analog_i2c_nack(analog_i2c_t *analog_i2c);
bool analog_i2c_send_data(analog_i2c_t *analog_i2c, uint8_t data);
void analog_i2c_read_data(analog_i2c_t *analog_i2c, uint8_t *data, bool last);
void i2c_sda_toggle_tx(analog_i2c_t *analog_i2c, uint8_t en);
#endif
