#ifndef APPLICATION_PROCESS_H
#define APPLICATION_PROCESS_H

#include <stdint.h>

void bilateral_filter(uint8_t *in, uint8_t *out);
void bilateral_filter_u16_rect(const uint16_t *in, uint16_t *out,
                               uint8_t rows, uint8_t cols, uint8_t stride);
uint16_t border_default(const uint8_t *frame, int i, int j);
#endif // APPLICATION_PROCESS_H

