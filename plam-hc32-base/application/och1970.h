#ifndef __OCH1970_H
#define __OCH1970_H

#include "analog_i2c.h"

#include <stdbool.h>
#include <stdint.h>

#define OCH1970_I2C_ADDR 0x0D

#define OCH1970_REG_WIA   0x00 // 设备ID寄存器
#define OCH1970_REG_ST    0x10 // 状态寄存器
#define OCH1970_REG_HX    0x11 // X轴数据寄存器
#define OCH1970_REG_HY    0x12 // Y轴数据寄存器
#define OCH1970_REG_HZ    0x14 // Z轴数据寄存器
#define OCH1970_REG_HXYZ  0x17 // XYZ轴数据寄存器
#define OCH1970_REG_CNTL1 0x20 // 控制寄存器1
#define OCH1970_REG_CNTL2 0x21 // 控制寄存器2
#define OCH1970_REG_BOP1X 0x22 // X轴阈值1设置
#define OCH1970_REG_BOP2X 0x23 // X轴阈值2设置
#define OCH1970_REG_BOP1Y 0x24 // Y轴阈值1设置
#define OCH1970_REG_BOP2Y 0x25 // Y轴阈值2设置
#define OCH1970_REG_BOP1Z 0x26 // Z轴阈值1设置
#define OCH1970_REG_BOP2Z 0x27 // Z轴阈值2设置
#define OCH1970_REG_SRST  0x30 // 软复位寄存器

typedef enum
{
    OCH1970_MODE_POWER_DOWN = 0x00,
    OCH1970_MODE_SINGLE     = 0x01, // 单次测量模式
    OCH1970_MODE_CONT_0_5HZ = 0x02,
    OCH1970_MODE_CONT_1HZ   = 0x04,
    OCH1970_MODE_CONT_2HZ   = 0x06,
    OCH1970_MODE_CONT_20HZ  = 0x08,
    OCH1970_MODE_CONT_40HZ  = 0x0A,
    OCH1970_MODE_CONT_100HZ = 0x0C,
    OCH1970_MODE_CONT_500HZ = 0x0E
} och1970_mode_t;

typedef enum
{
    OCH1970_DRIVE_LOW_NOISE = 0,
    OCH1970_DRIVE_LOW_POWER = 1
} och1970_drive_t;

typedef enum
{
    OCH1970_RANGE_HIGH_SENS = 0, // 高灵敏度模式 1.1ut
    OCH1970_RANGE_WIDE      = 1  // 宽范围模式 Z=3.1ut
} och1970_range_t;

typedef struct
{
    uint16_t state;
    int16_t  x;
    int16_t  y;
    int16_t  z;
    uint8_t  megnetic_buf[8];
} och1970_data_t;

typedef struct
{
    analog_i2c_t    i2c;
    och1970_mode_t  mode;
    och1970_drive_t drive;
    och1970_range_t range;
    och1970_data_t  magnetic_data;
} och1970_t;

bool och1970_init(och1970_t *dev);
bool och1970_reset(och1970_t *dev);
bool och1970_set_mode(och1970_t *deve);
bool och1970_set_drive(och1970_t *dev);
bool och1970_set_range(och1970_t *dev);
bool och1970_read_data(och1970_t *dev);

bool och1970_read_status(och1970_t *dev, uint16_t *status);
bool och1970_check_device_id(och1970_t *dev);
#endif
