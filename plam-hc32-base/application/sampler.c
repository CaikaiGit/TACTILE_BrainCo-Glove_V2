#include "sampler.h"

#include "main.h"
#include "process.h"
#include "time.h"

typedef struct
{
    uint8_t u8Port;
    uint16_t u16Pin;
} app_pin_t;

typedef struct
{
    uint8_t id;
    uint8_t y_start;
    uint8_t y_count;
} sampler_region_t;

#define SAMPLER_SETTLE_US     (5U)
#define APP_ADC_UNIT          (CM_ADC1)
#define APP_ADC_SEQ           (ADC_SEQ_A)
#define APP_ADC_EOC_FLAG      (ADC_FLAG_EOCA)
#define APP_ADC_TIMEOUT       (512UL)
#define APP_ADC_FCG_ENABLE()  (FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE))
#define APP_ADC_PGA_UNIT      (ADC_PGA1)
#define APP_ADC_PGA_GAIN      (ADC_PGA_GAIN_4)
#define APP_ADC_PGA_VSS       (ADC_PGA_VSS_AVSS)
#define SAMPLER_COLUMN_NONE   (0xFFU)
#define APP_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

/* All five analogue switches share AD0, mapped to ADC1_IN0 / PA0. */
static const app_pin_t m_stcAdcPin = { GPIO_PORT_A, GPIO_PIN_00 };

/* Each V2 finger has its own Y GPIO bank. Unused pinky entries are never read. */
static const app_pin_t m_astcColumnPins[SAMPLER_FINGER_COUNT][SAMPLER_MAX_Y_COUNT] = {
    { { GPIO_PORT_A, GPIO_PIN_10 }, { GPIO_PORT_D, GPIO_PIN_11 }, { GPIO_PORT_A, GPIO_PIN_09 }, { GPIO_PORT_D, GPIO_PIN_10 }, { GPIO_PORT_C, GPIO_PIN_09 }, { GPIO_PORT_D, GPIO_PIN_12 }, { GPIO_PORT_C, GPIO_PIN_06 }, { GPIO_PORT_D, GPIO_PIN_13 }, { GPIO_PORT_C, GPIO_PIN_07 }, { GPIO_PORT_D, GPIO_PIN_14 }, { GPIO_PORT_C, GPIO_PIN_08 } },
    { { GPIO_PORT_E, GPIO_PIN_08 }, { GPIO_PORT_B, GPIO_PIN_10 }, { GPIO_PORT_E, GPIO_PIN_09 }, { GPIO_PORT_E, GPIO_PIN_14 }, { GPIO_PORT_E, GPIO_PIN_10 }, { GPIO_PORT_E, GPIO_PIN_15 }, { GPIO_PORT_E, GPIO_PIN_11 }, { GPIO_PORT_B, GPIO_PIN_12 }, { GPIO_PORT_E, GPIO_PIN_12 }, { GPIO_PORT_B, GPIO_PIN_13 }, { GPIO_PORT_E, GPIO_PIN_13 } },
    { { GPIO_PORT_A, GPIO_PIN_01 }, { GPIO_PORT_B, GPIO_PIN_02 }, { GPIO_PORT_A, GPIO_PIN_02 }, { GPIO_PORT_E, GPIO_PIN_07 }, { GPIO_PORT_A, GPIO_PIN_04 }, { GPIO_PORT_B, GPIO_PIN_01 }, { GPIO_PORT_A, GPIO_PIN_03 }, { GPIO_PORT_C, GPIO_PIN_05 }, { GPIO_PORT_A, GPIO_PIN_07 }, { GPIO_PORT_B, GPIO_PIN_00 }, { GPIO_PORT_A, GPIO_PIN_06 } },
    { { GPIO_PORT_E, GPIO_PIN_04 }, { GPIO_PORT_C, GPIO_PIN_00 }, { GPIO_PORT_B, GPIO_PIN_09 }, { GPIO_PORT_C, GPIO_PIN_01 }, { GPIO_PORT_C, GPIO_PIN_13 }, { GPIO_PORT_B, GPIO_PIN_05 }, { GPIO_PORT_E, GPIO_PIN_05 }, { GPIO_PORT_H, GPIO_PIN_02 }, { GPIO_PORT_C, GPIO_PIN_14 }, { GPIO_PORT_C, GPIO_PIN_15 }, { GPIO_PORT_E, GPIO_PIN_06 } },
    { { GPIO_PORT_C, GPIO_PIN_12 }, { GPIO_PORT_B, GPIO_PIN_08 }, { GPIO_PORT_A, GPIO_PIN_15 }, { GPIO_PORT_D, GPIO_PIN_07 }, { GPIO_PORT_D, GPIO_PIN_01 }, { GPIO_PORT_D, GPIO_PIN_04 }, { GPIO_PORT_B, GPIO_PIN_03 }, { GPIO_PORT_B, GPIO_PIN_04 }, { GPIO_PORT_D, GPIO_PIN_03 }, { 0U, 0U }, { 0U, 0U } },
};

static const uint8_t m_au8FingerYCounts[SAMPLER_FINGER_COUNT] = { 11U, 11U, 11U, 11U, 9U };

static const app_pin_t m_astcMuxAddressPins[3U] = {
    { GPIO_PORT_E, GPIO_PIN_01 }, /* AS_S0 */
    { GPIO_PORT_D, GPIO_PIN_06 }, /* AS_S1 */
    { GPIO_PORT_D, GPIO_PIN_09 }, /* AS_S2 */
};

/* Active-low mux enables, in protocol part order. */
static const app_pin_t m_astcFingerEnablePins[SAMPLER_FINGER_COUNT] = {
    { GPIO_PORT_D, GPIO_PIN_08 }, /* EN0: thumb  */
    { GPIO_PORT_C, GPIO_PIN_11 }, /* EN1: index  */
    { GPIO_PORT_C, GPIO_PIN_04 }, /* EN2: middle */
    { GPIO_PORT_A, GPIO_PIN_05 }, /* EN3: ring   */
    { GPIO_PORT_C, GPIO_PIN_03 }, /* EN4: pinky  */
};

/*
 * One byte per Y line, bit X set when the X/Y crossing is a real sensor pad.
 * These masks come from matching PAD.netName intersections in O02.eprj2.
 */
static const uint8_t m_au8PointMask[SAMPLER_FINGER_COUNT][SAMPLER_MAX_Y_COUNT] = {
    { 0x18U, 0x18U, 0x91U, 0x91U, 0x11U, 0xD3U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU },
    { 0x0CU, 0x0CU, 0x08U, 0x49U, 0x49U, 0x48U, 0xFFU, 0x3FU, 0x3FU, 0xFFU, 0xFFU },
    { 0x0CU, 0x0CU, 0x08U, 0x49U, 0x49U, 0x48U, 0xFFU, 0x3FU, 0x3FU, 0xFFU, 0xFFU },
    { 0x0CU, 0x0CU, 0x08U, 0x49U, 0x49U, 0x48U, 0xFFU, 0x3FU, 0x3FU, 0xFFU, 0xFFU },
    { 0x0CU, 0x08U, 0x29U, 0x29U, 0x28U, 0xFFU, 0x3FU, 0x3FU, 0x3FU, 0x00U, 0x00U },
};

/* Region boundaries follow the clear gaps between pad clusters. */
static const sampler_region_t m_astcThumbRegions[] = {
    { SAMPLER_REGION_TIP,    0U, 7U },
    { SAMPLER_REGION_MIDDLE, 7U, 4U },
};

static const sampler_region_t m_astcFingerRegions[] = {
    { SAMPLER_REGION_TIP,    0U, 7U },
    { SAMPLER_REGION_MIDDLE, 7U, 3U },
    { SAMPLER_REGION_ROOT,  10U, 1U },
};

static const sampler_region_t m_astcPinkyRegions[] = {
    { SAMPLER_REGION_TIP,    0U, 7U },
    { SAMPLER_REGION_MIDDLE, 7U, 2U },
};

static uint8_t m_u8CurrentFinger;
static uint8_t m_u8CurrentColumn;

static void sampler_config_output_pin(uint8_t u8Port, uint16_t u16Pin, en_pin_state_t enState)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinState = (uint16_t)enState;
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinOutputType = PIN_OUT_TYPE_CMOS;
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    stcGpioInit.u16PullUp = PIN_PU_OFF;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(u8Port, u16Pin, &stcGpioInit);
}

static void sampler_write_pin(uint8_t u8Port, uint16_t u16Pin, en_pin_state_t enState)
{
    if (enState == PIN_STAT_SET) {
        GPIO_SetPins(u8Port, u16Pin);
    } else {
        GPIO_ResetPins(u8Port, u16Pin);
    }
}

static void sampler_set_all_cols(en_pin_state_t enState)
{
    uint8_t finger;
    uint8_t y;

    for (finger = 0U; finger < SAMPLER_FINGER_COUNT; ++finger) {
        for (y = 0U; y < m_au8FingerYCounts[finger]; ++y) {
            sampler_write_pin(m_astcColumnPins[finger][y].u8Port,
                              m_astcColumnPins[finger][y].u16Pin, enState);
        }
    }
    m_u8CurrentColumn = SAMPLER_COLUMN_NONE;
}

static void sampler_set_mux_address(uint8_t u8Address)
{
    uint8_t i;

    for (i = 0U; i < APP_ARRAY_SIZE(m_astcMuxAddressPins); ++i) {
        sampler_write_pin(m_astcMuxAddressPins[i].u8Port, m_astcMuxAddressPins[i].u16Pin,
                          ((u8Address & (1U << i)) != 0U) ? PIN_STAT_SET : PIN_STAT_RST);
    }
}

static void sampler_select_finger(uint8_t finger)
{
    uint8_t i;

    for (i = 0U; i < SAMPLER_FINGER_COUNT; ++i) {
        sampler_write_pin(m_astcFingerEnablePins[i].u8Port,
                          m_astcFingerEnablePins[i].u16Pin, PIN_STAT_SET);
    }

    m_u8CurrentColumn = SAMPLER_COLUMN_NONE;
    if (finger < SAMPLER_FINGER_COUNT) {
        sampler_write_pin(m_astcFingerEnablePins[finger].u8Port,
                          m_astcFingerEnablePins[finger].u16Pin, PIN_STAT_RST);
        m_u8CurrentFinger = finger;
    } else {
        m_u8CurrentFinger = SAMPLER_FINGER_COUNT;
    }
}

static uint16_t sampler_read_adc(void)
{
    uint32_t timeout = 0UL;

    ADC_ClearStatus(APP_ADC_UNIT, APP_ADC_EOC_FLAG);
    (void)ADC_Start(APP_ADC_UNIT);
    while (ADC_GetStatus(APP_ADC_UNIT, APP_ADC_EOC_FLAG) != SET) {
        if (timeout++ >= APP_ADC_TIMEOUT) {
            ADC_Stop(APP_ADC_UNIT);
            return 0U;
        }
    }

    ADC_ClearStatus(APP_ADC_UNIT, APP_ADC_EOC_FLAG);
    return ADC_GetValue(APP_ADC_UNIT, ADC_CH0);
}

uint8_t sampler_rect_skip_point(uint8_t rect_id, uint8_t x, uint8_t y)
{
    uint8_t finger;

    if ((rect_id < SAMPLER_RECT_THUMB) || (rect_id > SAMPLER_RECT_PINKY) ||
        (x >= SAMPLER_X_COUNT) || (y >= SAMPLER_MAX_Y_COUNT)) {
        return 1U;
    }

    finger = (uint8_t)(rect_id - SAMPLER_RECT_THUMB);
    if (y >= m_au8FingerYCounts[finger]) {
        return 1U;
    }
    return ((m_au8PointMask[finger][y] & (uint8_t)(1U << x)) == 0U) ? 1U : 0U;
}

void sampler_hardware_init(void)
{
    uint8_t i;
    stc_gpio_init_t stcGpioInit;
    stc_adc_init_t stcAdcInit;

    GPIO_SetDebugPort((GPIO_PIN_TDI | GPIO_PIN_TDO | GPIO_PIN_TRST), DISABLE);
    GPIO_AnalogCmd(GPIO_PORT_C, GPIO_PIN_14 | GPIO_PIN_15, DISABLE);

    APP_ADC_FCG_ENABLE();
    (void)ADC_StructInit(&stcAdcInit);
    (void)ADC_Init(APP_ADC_UNIT, &stcAdcInit);

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(m_stcAdcPin.u8Port, m_stcAdcPin.u16Pin, &stcGpioInit);
    ADC_ChCmd(APP_ADC_UNIT, APP_ADC_SEQ, ADC_CH0, ENABLE);
    ADC_SetSampleTime(APP_ADC_UNIT, ADC_CH0, 0x20U);

    ADC_PGA_Config(APP_ADC_UNIT, APP_ADC_PGA_UNIT, APP_ADC_PGA_GAIN, APP_ADC_PGA_VSS);
    ADC_PGA_SelectInputSrc(APP_ADC_UNIT, ADC_PGA_PIN_ADC1_PA0);
    ADC_PGA_Cmd(APP_ADC_UNIT, APP_ADC_PGA_UNIT, ENABLE);

    for (i = 0U; i < APP_ARRAY_SIZE(m_astcMuxAddressPins); ++i) {
        sampler_config_output_pin(m_astcMuxAddressPins[i].u8Port,
                                  m_astcMuxAddressPins[i].u16Pin, PIN_STAT_RST);
    }
    for (i = 0U; i < APP_ARRAY_SIZE(m_astcFingerEnablePins); ++i) {
        sampler_config_output_pin(m_astcFingerEnablePins[i].u8Port,
                                  m_astcFingerEnablePins[i].u16Pin, PIN_STAT_SET);
    }
    for (i = 0U; i < SAMPLER_FINGER_COUNT; ++i) {
        uint8_t y;
        for (y = 0U; y < m_au8FingerYCounts[i]; ++y) {
            sampler_config_output_pin(m_astcColumnPins[i][y].u8Port,
                                      m_astcColumnPins[i][y].u16Pin, PIN_STAT_RST);
        }
    }

    sampler_set_all_cols(PIN_STAT_RST);
    m_u8CurrentFinger = SAMPLER_FINGER_COUNT;
}

void sampler_switch_rows(uint8_t N)
{
    if (N < SAMPLER_FINGER_COUNT) {
        sampler_select_finger(N);
    }
}

void sampler_switch_cols(uint8_t N)
{
    if ((m_u8CurrentFinger >= SAMPLER_FINGER_COUNT) ||
        (N >= m_au8FingerYCounts[m_u8CurrentFinger])) {
        return;
    }

    if (m_u8CurrentColumn == N) {
        return;
    }
    if (m_u8CurrentColumn < m_au8FingerYCounts[m_u8CurrentFinger]) {
        sampler_write_pin(m_astcColumnPins[m_u8CurrentFinger][m_u8CurrentColumn].u8Port,
                          m_astcColumnPins[m_u8CurrentFinger][m_u8CurrentColumn].u16Pin, PIN_STAT_RST);
    }
    sampler_write_pin(m_astcColumnPins[m_u8CurrentFinger][N].u8Port,
                      m_astcColumnPins[m_u8CurrentFinger][N].u16Pin, PIN_STAT_SET);
    m_u8CurrentColumn = N;
}

uint16_t sampler_get_value(void)
{
    return (m_u8CurrentFinger < SAMPLER_FINGER_COUNT) ? sampler_read_adc() : 0U;
}

void sampler_capture_rectangles(sampler_rectangles_t *rects)
{
    uint8_t x;
    uint8_t y;
    uint8_t finger;
    uint8_t point_mask;

    if (rects == NULL) {
        return;
    }

    /* Select exactly one active-low finger enable before scanning its own Y bank. */
    for (finger = 0U; finger < SAMPLER_FINGER_COUNT; ++finger) {
        sampler_set_all_cols(PIN_STAT_RST);
        sampler_select_finger(finger);
#if SAMPLER_SETTLE_US > 0U
        delay_us(SAMPLER_SETTLE_US);
#endif
        for (y = 0U; y < m_au8FingerYCounts[finger]; ++y) {
            point_mask = m_au8PointMask[finger][y];
            if (point_mask == 0U) {
                for (x = 0U; x < SAMPLER_X_COUNT; ++x) {
                    rects->finger[finger][y][x] = 0U;
                }
                continue;
            }

            sampler_switch_cols(y);
            for (x = 0U; x < SAMPLER_X_COUNT; ++x) {
                if ((point_mask & (uint8_t)(1U << x)) == 0U) {
                    rects->finger[finger][y][x] = 0U;
                    continue;
                }
                sampler_set_mux_address(x);
#if SAMPLER_SETTLE_US > 0U
                delay_us(SAMPLER_SETTLE_US);
#endif
                rects->finger[finger][y][x] = sampler_read_adc();
            }
        }
    }

    sampler_set_all_cols(PIN_STAT_RST);
    sampler_select_finger(SAMPLER_FINGER_COUNT);
}

uint8_t sampler_get_rect_view(const sampler_rectangles_t *rects, uint8_t rect_id,
                              sampler_rect_view_t *view)
{
    uint8_t finger;

    if ((rects == NULL) || (view == NULL) ||
        (rect_id < SAMPLER_RECT_THUMB) || (rect_id > SAMPLER_RECT_PINKY)) {
        return 0U;
    }

    finger = (uint8_t)(rect_id - SAMPLER_RECT_THUMB);
    view->id = rect_id;
    view->rows = SAMPLER_Y_COUNT;
    view->cols = SAMPLER_X_COUNT;
    view->data = &rects->finger[finger][0][0];
    view->stride = SAMPLER_X_COUNT;
    return 1U;
}

static uint16_t sampler_pack_regions(const sampler_rectangles_t *rects, uint8_t rect_id,
                                     const sampler_region_t *regions, uint8_t region_count,
                                     uint8_t *payload, uint16_t payload_size)
{
    uint16_t required = 1U;
    uint16_t offset = 1U;
    uint8_t region_index;
    uint8_t finger = (uint8_t)(rect_id - SAMPLER_RECT_THUMB);
    uint8_t valid_counts[SAMPLER_FINGER_MAX_REGION_COUNT] = { 0U };

    if (region_count > SAMPLER_FINGER_MAX_REGION_COUNT) {
        return 0U;
    }

    for (region_index = 0U; region_index < region_count; ++region_index) {
        uint8_t x;
        uint8_t y_offset;
        uint8_t valid_count = 0U;

        for (y_offset = 0U; y_offset < regions[region_index].y_count; ++y_offset) {
            uint8_t y = (uint8_t)(regions[region_index].y_start + y_offset);
            for (x = 0U; x < SAMPLER_X_COUNT; ++x) {
                if (sampler_rect_skip_point(rect_id, x, y) == 0U) {
                    ++valid_count;
                }
            }
        }
        valid_counts[region_index] = valid_count;
        required = (uint16_t)(required + SAMPLER_REGION_HEADER_SIZE +
                   ((uint16_t)valid_count * (uint16_t)FRAME_DATA_BYTES));
    }
    if (payload_size < required) {
        return 0U;
    }

    payload[0] = (uint8_t)((rect_id << 4U) | region_count);
    for (region_index = 0U; region_index < region_count; ++region_index) {
        uint8_t x;
        uint8_t y_offset;
        uint8_t valid_count = valid_counts[region_index];

        payload[offset++] = regions[region_index].id;
        payload[offset++] = valid_count;
        payload[offset++] = 1U;

        /* Compact fixed-map ordering: Y outside, X inside; invalid crossings are omitted. */
        for (y_offset = 0U; y_offset < regions[region_index].y_count; ++y_offset) {
            uint8_t y = (uint8_t)(regions[region_index].y_start + y_offset);
            for (x = 0U; x < SAMPLER_X_COUNT; ++x) {
                uint16_t value = rects->finger[finger][y][x];
                if (sampler_rect_skip_point(rect_id, x, y) != 0U) {
                    continue;
                }
                payload[offset++] = (uint8_t)(value & 0xFFU);
#if FRAME_DATA_BYTES == 2U
                payload[offset++] = (uint8_t)(value >> 8U);
#endif
            }
        }
    }

    return offset;
}

uint16_t sampler_pack_finger_payload(const sampler_rectangles_t *rects, uint8_t rect_id,
                                     uint8_t *payload, uint16_t payload_size)
{
    if ((rects == NULL) || (payload == NULL)) {
        return 0U;
    }

    switch (rect_id) {
    case SAMPLER_RECT_THUMB:
        return sampler_pack_regions(rects, rect_id, m_astcThumbRegions,
                                    (uint8_t)APP_ARRAY_SIZE(m_astcThumbRegions),
                                    payload, payload_size);
    case SAMPLER_RECT_INDEX:
    case SAMPLER_RECT_MIDDLE:
    case SAMPLER_RECT_RING:
        return sampler_pack_regions(rects, rect_id, m_astcFingerRegions,
                                    (uint8_t)APP_ARRAY_SIZE(m_astcFingerRegions),
                                    payload, payload_size);
    case SAMPLER_RECT_PINKY:
        return sampler_pack_regions(rects, rect_id, m_astcPinkyRegions,
                                    (uint8_t)APP_ARRAY_SIZE(m_astcPinkyRegions),
                                    payload, payload_size);
    default:
        return 0U;
    }
}

void sampler_get_frame(uint8_t *buffer)
{
    uint8_t y;
    uint8_t finger;

    if (buffer == NULL) {
        return;
    }

    for (finger = 0U; finger < SAMPLER_FINGER_COUNT; ++finger) {
        sampler_switch_rows(finger);
        for (y = 0U; y < SAMPLER_Y_COUNT; ++y) {
            uint16_t point = ((uint16_t)y * SAMPLER_FINGER_COUNT) + finger;
            uint16_t value = 0U;

            if ((y < m_au8FingerYCounts[finger]) &&
                ((m_au8PointMask[finger][y] & 0x01U) != 0U)) {
                sampler_switch_cols(y);
                sampler_set_mux_address(0U);
#if SAMPLER_SETTLE_US > 0U
                delay_us(SAMPLER_SETTLE_US);
#endif
                value = sampler_read_adc();
            }
#if FRAME_DATA_BYTES == 1U
            buffer[point] = (uint8_t)(value >> 4U);
#else
            buffer[2U * point] = (uint8_t)(value & 0xFFU);
            buffer[(2U * point) + 1U] = (uint8_t)(value >> 8U);
#endif
        }
    }
    sampler_set_all_cols(PIN_STAT_RST);
    sampler_select_finger(SAMPLER_FINGER_COUNT);
}

void pressure_transform_calibration(uint8_t *in, uint8_t *out,
                                    uint64_t ad_value_sum, int pressure_sum,
                                    uint8_t start_col, uint8_t end_col,
                                    uint8_t start_row, uint8_t end_row)
{
    int i;
    int j;

    for (i = start_col; i <= end_col; ++i) {
        for (j = start_row; j <= end_row; ++j) {
            uint16_t ad_value = border_default(in, i, j);
            float pressure;
            int pressure_int;

            if (ad_value_sum != 0U) {
                pressure = (float)(ad_value * pressure_sum) / ad_value_sum;
            } else {
                pressure = 0.0f;
            }

            if (pressure < 0.0f) {
                pressure_int = 0;
            } else if (pressure > 65535.0f) {
                pressure_int = 65535;
            } else {
                pressure_int = (int)(pressure + 0.5f);
            }

            out[2 * (i * ROWS + j)] = pressure_int & 0xFF;
            out[2 * (i * ROWS + j) + 1] = (pressure_int >> 8) & 0xFF;
        }
    }
}
