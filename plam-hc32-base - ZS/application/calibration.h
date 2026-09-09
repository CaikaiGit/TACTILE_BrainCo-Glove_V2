#ifndef APPLICATION_CALIBRATION_H
#define APPLICATION_CALIBRATION_H

#include "configuration.h"
// #include <stdint.h>

enum adaptive_t
{
    ADAPTIVE_MAX  = 0x01,
    ADAPTIVE_MEAN = 0x02,
};

enum threshold_t
{
    THRESHOLD_TRUNC  = 0x01,
    THRESHOLD_TOZERO = 0x02,
};

// typedef struct
//{
//     uint8_t volatile enabled;
//     uint8_t  method;
//     uint16_t frames;
//     float    alpha;
//     uint16_t static_beta;
//     float    dynamic_beta;
//     uint8_t  type;
//     uint8_t volatile finished;
// } zeroing_t;

void calibration_enable(config_t *params, uint8_t en);

void calibration_feed(config_t *params);

void calibration_zero(config_t *params, uint8_t *buffer);

void calibration_clear(config_t *params);

#endif // APPLICATION_CALIBRATION_H
