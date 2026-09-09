/**
 * @file    och1970.c
 * @brief   OCH1970WPAD-H 3D磁传感器驱动实现
 * @note
 */

#include "och1970.h"

#include "analog_i2c.h"
#include "time.h"

static bool och1970_write_reg(och1970_t *dev, uint8_t reg, uint8_t *data, uint8_t len);
static bool och1970_read_reg(och1970_t *dev, uint8_t reg, uint8_t *data, uint8_t len);

bool och1970_init(och1970_t *dev)
{
    if (!och1970_reset(dev)) {
        return false;
    }

    if (!och1970_check_device_id(dev)) {
        return false;
    }
    if (!och1970_set_mode(dev)) {
        return false;
    }
    if (!och1970_set_drive(dev)) {
        return false;
    }

    if (!och1970_set_range(dev)) {
        return false;
    }
    return true;
}

bool och1970_reset(och1970_t *dev)
{
    uint8_t srst = 0x01;
    if (!och1970_write_reg(dev, OCH1970_REG_SRST, &srst, 1)) {
        return false;
    }
    delay_us(300000);
    return true;
}
bool och1970_check_device_id(och1970_t *dev)
{
    uint8_t wia[4] = { 0 };
    memset(wia, 0, 4);
    if (!och1970_read_reg(dev, OCH1970_REG_WIA, wia, 4)) {
        return false;
    }

    if (wia[0] != 0x48 || wia[1] != 0xC0) {
        return false;
    }
    return true;
}

bool och1970_set_mode(och1970_t *dev)
{
    uint8_t cntl2[1] = { 0 };

    if (!och1970_read_reg(dev, OCH1970_REG_CNTL2, cntl2, 1)) {
        return false;
    }
    cntl2[0] &= 0xF0;
    cntl2[0] |= (dev->mode & 0x0F);
    if (!och1970_write_reg(dev, OCH1970_REG_CNTL2, cntl2, 1)) {
        return false;
    }
    return true;
}

bool och1970_set_drive(och1970_t *dev)
{
    uint8_t cntl2[1] = { 0 };

    if (!och1970_read_reg(dev, OCH1970_REG_CNTL2, cntl2, 1)) {
        return false;
    }

    cntl2[0] &= ~(1 << 4);
    cntl2[0] |= (dev->drive << 4);

    if (!och1970_write_reg(dev, OCH1970_REG_CNTL2, cntl2, 1)) {
        return false;
    }
    return true;
}

bool och1970_set_range(och1970_t *dev)
{
    uint8_t cntl2[1] = { 0 };
    if (!och1970_read_reg(dev, OCH1970_REG_CNTL2, cntl2, 1)) {
        return false;
    }
    cntl2[0] &= ~(1 << 5);
    cntl2[0] |= (dev->range << 5);

    if (!och1970_write_reg(dev, OCH1970_REG_CNTL2, cntl2, 1)) {
        return false;
    }
    return true;
}

bool och1970_read_status(och1970_t *dev, uint16_t *status)
{
    uint8_t st[2] = { 0 };

    if (!och1970_read_reg(dev, OCH1970_REG_ST, st, 2)) {
        return false;
    }

    *status = (st[0] << 8) | st[1];
    return true;
}

static bool och1970_write_reg(och1970_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    analog_i2c_start(&dev->i2c);

    if (!analog_i2c_send_data(&dev->i2c, OCH1970_I2C_ADDR << 1)) return false;
    if (!analog_i2c_send_data(&dev->i2c, reg)) return false;

    for (uint8_t i = 0; i < len; i++) {
        if (!analog_i2c_send_data(&dev->i2c, data[i])) return false;
    }
    analog_i2c_stop(&dev->i2c);

    return true;
}

static bool och1970_read_reg(och1970_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    analog_i2c_start(&dev->i2c);

    if (!analog_i2c_send_data(&dev->i2c, OCH1970_I2C_ADDR << 1)) return false;

    if (!analog_i2c_send_data(&dev->i2c, reg)) return false;

    analog_i2c_start(&dev->i2c);

    if (!analog_i2c_send_data(&dev->i2c, (OCH1970_I2C_ADDR << 1) | 0x01)) return false;

    for (uint8_t i = 0; i < len; i++) {
        if (i == len - 1) {
            analog_i2c_read_data(&dev->i2c, &data[i], true);
        }
        else {
            analog_i2c_read_data(&dev->i2c, &data[i], false);
        }
    }
    analog_i2c_stop(&dev->i2c);
    return true;
}

bool och1970_read_data(och1970_t *dev)
{
    uint16_t status = 0;

    if (!och1970_read_status(dev, &status)) {
        return false;
    }

    if (!(status & 0x0001)) {
        return false;
    }

    if (!och1970_read_reg(dev, OCH1970_REG_HXYZ, dev->magnetic_data.megnetic_buf, 8)) {
        return false;
    }
    dev->magnetic_data.state =
        (dev->magnetic_data.megnetic_buf[0] << 8) | dev->magnetic_data.megnetic_buf[1];
    dev->magnetic_data.z = (dev->magnetic_data.megnetic_buf[2] << 8) | dev->magnetic_data.megnetic_buf[3];
    dev->magnetic_data.y = (dev->magnetic_data.megnetic_buf[4] << 8) | dev->magnetic_data.megnetic_buf[5];
    dev->magnetic_data.x = (dev->magnetic_data.megnetic_buf[6] << 8) | dev->magnetic_data.megnetic_buf[7];
    return true;
}
