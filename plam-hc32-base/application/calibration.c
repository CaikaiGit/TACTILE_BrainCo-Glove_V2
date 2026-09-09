#include "calibration.h"

void calibration_enable(config_t *params, uint8_t en) { params->zeroing.enabled = en; }

void calibration_feed(config_t *params)
{
    (void)params;
}

void calibration_zero(config_t *params, uint8_t *frame)
{
    (void)params;
    (void)frame;
}

void calibration_clear(config_t *params)
{
    params->zeroing.finished = 0x00;
}
