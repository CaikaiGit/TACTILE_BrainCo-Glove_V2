#include "process.h"

#include "sampler.h"
#include "utils.h"

#include <stdlib.h>

uint16_t border_default(const uint8_t *frame, int i, int j)
{
    i = CLAMP(i, 0, COLS - 1);
    j = CLAMP(j, 0, ROWS - 1);

	if(FRAME_DATA_BYTES == 1)
	{
		return frame[i * ROWS + j];
	}
	else if(FRAME_DATA_BYTES == 2)
	{
		return frame[2 * (i * ROWS + j)] | (frame[2 * (i * ROWS + j) + 1] << 8);
	}
}

static uint16_t border_default_u16_rect(const uint16_t *frame, uint8_t rows,
                                        uint8_t cols, uint8_t stride,
                                        int x, int y)
{
    x = CLAMP(x, 0, cols - 1);
    y = CLAMP(y, 0, rows - 1);

    return frame[((uint16_t)y * stride) + x];
}

static inline uint16_t gain(uint16_t value, uint16_t original, uint8_t alpha)
{
    if (value == 0) return 0;
    
    uint16_t max_value = MAX(original, value);
    uint16_t dis = abs(original - value);
    
    return value * alpha * (((max_value - dis) * 6) / max_value) / 6;
}

void bilateral_filter_u16_rect(const uint16_t *in, uint16_t *out,
                               uint8_t rows, uint8_t cols, uint8_t stride)
{
    int x;
    int y;

    if ((in == NULL) || (out == NULL) || (rows == 0U) || (cols == 0U)) {
        return;
    }

    for (x = 0; x < cols; ++x) {
        for (y = 0; y < rows; ++y) {
            const uint16_t original = border_default_u16_rect(in, rows, cols, stride, x, y);
            int value = original * 8;

            value += gain(border_default_u16_rect(in, rows, cols, stride, x - 1, y), original, 2);
            value += gain(border_default_u16_rect(in, rows, cols, stride, x + 1, y), original, 2);
            value += gain(border_default_u16_rect(in, rows, cols, stride, x, y - 1), original, 2);
            value += gain(border_default_u16_rect(in, rows, cols, stride, x, y + 1), original, 2);

            value += gain(border_default_u16_rect(in, rows, cols, stride, x - 1, y - 1), original, 1);
            value += gain(border_default_u16_rect(in, rows, cols, stride, x + 1, y - 1), original, 1);
            value += gain(border_default_u16_rect(in, rows, cols, stride, x - 1, y + 1), original, 1);
            value += gain(border_default_u16_rect(in, rows, cols, stride, x + 1, y + 1), original, 1);

            out[((uint16_t)y * stride) + x] = (uint16_t)((value + 19) / 20);
        }
    }
}

void bilateral_filter(uint8_t *in, uint8_t *out)
{
    for (int i = 0; i < COLS; ++i) {
        for (int j = 0; j < ROWS; ++j) {

            const uint16_t original = border_default(in, i, j);

            int value = original * 8;

            value += gain(border_default(in, i - 1, j), original, 2); // 左
            value += gain(border_default(in, i + 1, j), original, 2); // 右
            value += gain(border_default(in, i, j - 1), original, 2); // 上
            value += gain(border_default(in, i, j + 1), original, 2); // 下

            value += gain(border_default(in, i - 1, j - 1), original, 1); // 左上
            value += gain(border_default(in, i + 1, j - 1), original, 1); // 右上
            value += gain(border_default(in, i - 1, j + 1), original, 1); // 左下
            value += gain(border_default(in, i + 1, j + 1), original, 1); // 右下

            value = (value + 19) / 20;
			if(FRAME_DATA_BYTES == 1)
			{
				out[i * ROWS + j] = value & 0xff;
			}
			else if(FRAME_DATA_BYTES == 2)
			{
				out[2 * (i * ROWS + j)]     = value & 0x00ff;
				out[2 * (i * ROWS + j) + 1] = (value >> 8) & 0x00ff;
			}
        }
    }
}
