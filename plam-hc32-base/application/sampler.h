#ifndef APPLICATION_SAMPLER_H
#define APPLICATION_SAMPLER_H

#include <stdint.h>

/*
 * Strong-brain glove topology, taken from O02.eprj2:
 *   - five fingers, each with X0..X7 through an analogue mux;
 *   - each finger has an independent Y drive bank (Y0..Y10; pinky Y0..Y8);
 *   - one ADC and one active-low mux enable per finger;
 *   - there is no palm sensor on this board.
 */
#define SAMPLER_X_COUNT       8U
#define SAMPLER_MAX_Y_COUNT   11U
#define SAMPLER_Y_COUNT       SAMPLER_MAX_Y_COUNT
#define SAMPLER_FINGER_COUNT  5U

/* This project currently builds the right glove. 0 remains available for a future left build. */
#ifdef HAND_RIGHT
	#define STRONG_GLOVE_HAND_RIGHT 1U
#else
	#define STRONG_GLOVE_HAND_RIGHT 0U
#endif

#define FRAME_DATA_BYTES 1U

/*
 * Xiaomi-compatible finger payload:
 *   [part/region-count]
 *   repeated [region-id][x-count][y-count][samples...]
 */
#define SAMPLER_REGION_HEADER_SIZE       3U
#define SAMPLER_FINGER_MAX_REGION_COUNT  3U
#define SAMPLER_RECT_MAX_POINTS          (SAMPLER_X_COUNT * SAMPLER_Y_COUNT)
#define SAMPLER_RECT_MAX_PAYLOAD_SIZE    (1U + \
    (SAMPLER_FINGER_MAX_REGION_COUNT * SAMPLER_REGION_HEADER_SIZE) + \
    (SAMPLER_RECT_MAX_POINTS * FRAME_DATA_BYTES))

/*
 * Legacy flat-frame definitions are retained for the old calibration command
 * path. The live upload path uses sampler_capture_rectangles().
 */
#define ROWS             SAMPLER_FINGER_COUNT
#define COLS             SAMPLER_Y_COUNT
#define FRAME_SIZE       (FRAME_DATA_BYTES * ROWS * COLS)
#define PACKAGE_SIZE     (FRAME_SIZE + 4U)
#define ROWS_SEND        ROWS
#define COLS_SEND        COLS

#define CALIBRATION_1_VALID_POINTS (ROWS * COLS)
#define CALIBRATION_1_START_ROWS   0U
#define CALIBRATION_1_START_COLS   0U
#define CALIBRATION_1_END_ROWS     (ROWS - 1U)
#define CALIBRATION_1_END_COLS     (COLS - 1U)
#define CALIBRATION_2_VALID_POINTS CALIBRATION_1_VALID_POINTS
#define CALIBRATION_2_START_ROWS   CALIBRATION_1_START_ROWS
#define CALIBRATION_2_START_COLS   CALIBRATION_1_START_COLS
#define CALIBRATION_2_END_ROWS     CALIBRATION_1_END_ROWS
#define CALIBRATION_2_END_COLS     CALIBRATION_1_END_COLS
#define CALIBRATION_3_VALID_POINTS CALIBRATION_1_VALID_POINTS
#define CALIBRATION_3_START_ROWS   CALIBRATION_1_START_ROWS
#define CALIBRATION_3_START_COLS   CALIBRATION_1_START_COLS
#define CALIBRATION_3_END_ROWS     CALIBRATION_1_END_ROWS
#define CALIBRATION_3_END_COLS     CALIBRATION_1_END_COLS
#define CALIBRATION_4_VALID_POINTS CALIBRATION_1_VALID_POINTS
#define CALIBRATION_4_START_ROWS   CALIBRATION_1_START_ROWS
#define CALIBRATION_4_START_COLS   CALIBRATION_1_START_COLS
#define CALIBRATION_4_END_ROWS     CALIBRATION_1_END_ROWS
#define CALIBRATION_4_END_COLS     CALIBRATION_1_END_COLS
#define FRAME_SEND_SIZE   (FRAME_DATA_BYTES * ROWS_SEND * COLS_SEND)
#define PACKAGE_SEND_SIZE (FRAME_SEND_SIZE + 4U)

typedef enum {
    SAMPLER_RECT_THUMB = 1,
    SAMPLER_RECT_INDEX = 2,
    SAMPLER_RECT_MIDDLE = 3,
    SAMPLER_RECT_RING = 4,
    SAMPLER_RECT_PINKY = 5,
    SAMPLER_RECT_COUNT = 5
} sampler_rect_id_t;

typedef enum {
    SAMPLER_REGION_TIP = 1,
    SAMPLER_REGION_MIDDLE = 2,
    SAMPLER_REGION_ROOT = 3
} sampler_region_id_t;

typedef struct {
    uint8_t id;
    uint8_t rows;
    uint8_t cols;
    const uint16_t *data;
    uint8_t stride;
} sampler_rect_view_t;

typedef struct {
    uint16_t finger[SAMPLER_FINGER_COUNT][SAMPLER_MAX_Y_COUNT][SAMPLER_X_COUNT];
} sampler_rectangles_t;

void sampler_hardware_init(void);
void sampler_switch_rows(uint8_t N);
void sampler_switch_cols(uint8_t N);
uint16_t sampler_get_value(void);

void sampler_get_frame(uint8_t *buffer);
void sampler_capture_rectangles(sampler_rectangles_t *rects);
uint8_t sampler_rect_skip_point(uint8_t rect_id, uint8_t x, uint8_t y);
uint8_t sampler_get_rect_view(const sampler_rectangles_t *rects, uint8_t rect_id,
                              sampler_rect_view_t *view);
uint16_t sampler_pack_finger_payload(const sampler_rectangles_t *rects, uint8_t rect_id,
                                     uint8_t *payload, uint16_t payload_size);

void pressure_transform_calibration(uint8_t *in, uint8_t *out,
                                    uint64_t ad_value_sum, int pressure_sum,
                                    uint8_t start_col, uint8_t end_col,
                                    uint8_t start_row, uint8_t end_row);

#endif
