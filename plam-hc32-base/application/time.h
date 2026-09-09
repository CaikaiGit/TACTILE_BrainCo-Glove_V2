#ifndef APPLICATION_TIME_H
#define APPLICATION_TIME_H

#include <stdint.h>

typedef void(TimeDelayUsCallback)(uint32_t us);

void time_set_delay_us_callback(TimeDelayUsCallback *callback);
void delay_us(uint32_t us);

#endif // APPLICATION_TIME_H
