#include "time.h"

static TimeDelayUsCallback *g_time_delay_us = 0;

void time_set_delay_us_callback(TimeDelayUsCallback *callback)
{
    g_time_delay_us = callback;
}

void delay_us(uint32_t us)
{
    if (g_time_delay_us != 0) {
        g_time_delay_us(us);
    }
}

