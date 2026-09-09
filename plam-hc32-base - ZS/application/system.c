#include "system.h"

#include "calibration.h"
#include "configuration.h"
#include "force_lut_calculator.h"
#include "force_lut_storage.h"
#include "internal_flash.h"
#include "process.h"
#include "time.h"
#include "sampler.h"
#include "serial.h"
#include "string.h"
#include "stdio.h"

/* The 200 Hz stream sends raw PGA/ADC samples; filtering remains available to legacy paths. */
#define RECT_REALTIME_FILTER_ENABLED 0U

// ======================= 外部变量声明 =======================
serial_t  serial1;
config_t config_;
uint8_t  package[PACKAGE_SEND_SIZE + 27] = { 0x01, 0x01, ROWS_SEND, COLS_SEND };
uint16_t  size              = 0;
bool  press_flag; // 压力模式标志位，false 上传AD值，true 上传压力值
volatile uint8_t package_id = 0xff;
static system_port_t g_system_port;
static uint8_t g_upload_cache[2][PACKAGE_SEND_SIZE + 27];
static volatile uint8_t g_upload_cache_active;
static volatile uint8_t g_upload_cache_ready;
static volatile uint16_t g_upload_cache_size;
static volatile uint8_t g_serial_dispatch_from_isr;
static volatile uint8_t g_serial_deferred_pending;
static package_t g_serial_deferred_package;
static volatile uint8_t g_rect_tx_paused;
static volatile uint8_t g_rect_tx_resume_after_tx;
static sampler_rectangles_t g_sampler_rects;
#if RECT_REALTIME_FILTER_ENABLED
static sampler_rectangles_t g_sampler_filter_rects;
#endif
static sampler_rectangles_t g_sampler_zero_rects;
static uint8_t g_rect_payload[SAMPLER_RECT_MAX_PAYLOAD_SIZE];
static uint8_t g_rect_zero_ready;

/* J-Link watch variables: completed UART traffic, not merely queued frames. */
volatile uint32_t g_debug_rect_tx_packets;
volatile uint32_t g_debug_hand_tx_count;
volatile uint32_t g_debug_hand_fps_milli;
volatile uint32_t g_debug_hand_window_ms;
static volatile uint8_t g_debug_rect_tx_active;
static uint8_t g_debug_rect_packets_in_hand;
static uint32_t g_debug_hands_in_window;
static uint32_t g_debug_hand_window_start_ms;

#define RECT_TX_QUEUE_CAPACITY 12U
typedef struct {
    uint8_t channel;
    uint8_t flags;
    uint16_t length;
    uint8_t payload[SAMPLER_RECT_MAX_PAYLOAD_SIZE];
} rect_tx_packet_t;

static rect_tx_packet_t g_rect_tx_queue[RECT_TX_QUEUE_CAPACITY];
static volatile uint8_t g_rect_tx_queue_head;
static volatile uint8_t g_rect_tx_queue_tail;
static volatile uint8_t g_rect_tx_queue_count;

#define RECT_TEST_ID 0U
#define RECT_SEND_COUNT ((RECT_TEST_ID == 0U) ? SAMPLER_RECT_COUNT : 1U)
#define RECT_OUTPUT_ID(rect_id) ((RECT_TEST_ID == 0U) ? 0U : 0U)

static const uint8_t g_rect_send_order[] = {
#if RECT_TEST_ID == 0U
    SAMPLER_RECT_THUMB,
    SAMPLER_RECT_INDEX,
    SAMPLER_RECT_MIDDLE,
    SAMPLER_RECT_RING,
    SAMPLER_RECT_PINKY,
#else
    RECT_TEST_ID,
#endif
};

// ======================= 内部静态数据 =======================
static uint8_t *pbuf;
// 压力标定数据
static float press_lut_1[2][PRESS_LUT_MAX_SIZE];
static float press_lut_2[2][PRESS_LUT_MAX_SIZE];
static float press_lut_3[2][PRESS_LUT_MAX_SIZE];
static float press_lut_4[2][PRESS_LUT_MAX_SIZE];
static uint8_t press_lut_cout_1 = 10;
static uint8_t press_lut_cout_2 = 10;
static uint8_t press_lut_cout_3 = 10;
static uint8_t press_lut_cout_4 = 10;

// ======================= 内部静态函数声明 =======================
static int findPosition(float arr[], uint8_t size, uint16_t target);
static void serial_toggle_tx(uint8_t en);
static void system_publish_upload_frame(const uint8_t *buffer, uint16_t buffer_size);
static void system_capture_rect_zero(void);
static void system_rect_accumulate_max_array(uint16_t *zero, const uint16_t *sample, uint16_t count);
static void system_rect_apply_zero_array(uint16_t *zero, uint16_t count);
static void system_rect_subtract_zero_array(uint16_t *sample, const uint16_t *zero, uint16_t count);
static void system_rect_accumulate_max(sampler_rectangles_t *zero, const sampler_rectangles_t *sample);
static void system_rect_apply_zero(sampler_rectangles_t *zero);
static void system_rect_subtract_zero(sampler_rectangles_t *sample, const sampler_rectangles_t *zero);
static void system_rect_normalize_sample(sampler_rectangles_t *sample);
static void system_rect_apply_floor(sampler_rectangles_t *sample);
#if RECT_REALTIME_FILTER_ENABLED
static void system_rect_filter(const sampler_rectangles_t *sample, sampler_rectangles_t *filtered);
#endif
static void system_rect_apply_force_lut(sampler_rectangles_t *rects);
static void system_rect_queue_clear(void);
static uint8_t system_rect_queue_free_count(void);
static uint8_t system_rect_queue_push(uint8_t ch, uint8_t flags, const uint8_t *payload, uint16_t length);
static uint8_t system_rect_queue_pop(rect_tx_packet_t *packet);
static void system_rect_tx_kick(void);
static void system_debug_rect_tx_complete(void);
static void system_queue_next_rect_frame(void);
static void system_service_rect_tx(void);
static void serial_handle_rx(uint8_t from_isr);
static void serial_dispatch_cmd(package_t *package);
static void serial_handle_deferred_cmd(void);
static uint8_t serial_cmd_is_irq_safe(const package_t *package);
static uint8_t serial_cmd_requires_tx_pause(const package_t *package);

// ======================= 串口管理模块 =======================
static void serial_init(void)
{
    serial1.uart         = g_system_port.serial_ctx;
    serial1.instance     = g_system_port.serial_ctx;
    serial1.dma_usart_rx = g_system_port.serial_ctx;
    serial1.write_ctx    = g_system_port.serial_ctx;
    serial1.onready      = serial_dispatch_cmd;
    serial1.toggle_tx    = serial_toggle_tx;
    serial1.get_tick     = g_system_port.get_tick;
    serial1.delay_ms     = g_system_port.delay_ms;
    serial1.write        = g_system_port.write;
    serial1.write_dma    = g_system_port.write_dma;
    serial1.flag         = 0;

    if (g_system_port.start_rx != NULL) {
        g_system_port.start_rx(g_system_port.serial_ctx, serial1.rxbuf, SERIAL_RX_BUFFER_SIZE);
    }
}

static void system_publish_upload_frame(const uint8_t *buffer, uint16_t buffer_size)
{
    uint8_t next_index;
    uint32_t primask;

    if ((buffer == NULL) || (buffer_size == 0U) || (buffer_size > sizeof(g_upload_cache[0]))) {
        return;
    }

    next_index = (uint8_t)(g_upload_cache_active ^ 1U);
    memcpy(g_upload_cache[next_index], buffer, buffer_size);

    primask = __get_PRIMASK();
    __disable_irq();
    g_upload_cache_size = buffer_size;
    g_upload_cache_active = next_index;
    g_upload_cache_ready = 1U;
    if (primask == 0U) {
        __enable_irq();
    }
}

static void system_rect_accumulate_max_array(uint16_t *zero, const uint16_t *sample, uint16_t count)
{
    uint16_t i;

    for (i = 0U; i < count; ++i) {
        if (sample[i] > zero[i]) {
            zero[i] = sample[i];
        }
    }
}

static void system_rect_apply_zero_array(uint16_t *zero, uint16_t count)
{
    uint16_t i;
    uint16_t value;

    for (i = 0U; i < count; ++i) {
        value = (uint16_t)(zero[i] * config_.zeroing.alpha) + config_.zeroing.static_beta;
        if (FRAME_DATA_BYTES == 1) {
            value = (uint16_t)(value >> 4);
        }
        zero[i] = value;
    }
}

static void system_rect_subtract_zero_array(uint16_t *sample, const uint16_t *zero, uint16_t count)
{
    uint16_t i;

    for (i = 0U; i < count; ++i) {
        sample[i] = (sample[i] > zero[i]) ? (uint16_t)(sample[i] - zero[i]) : 0U;
    }
}

static void system_rect_accumulate_max(sampler_rectangles_t *zero, const sampler_rectangles_t *sample)
{
    system_rect_accumulate_max_array(&zero->finger[0][0][0], &sample->finger[0][0][0],
                                     (uint16_t)(sizeof(*zero) / sizeof(uint16_t)));
}

static void system_rect_apply_zero(sampler_rectangles_t *zero)
{
    system_rect_apply_zero_array(&zero->finger[0][0][0],
                                 (uint16_t)(sizeof(*zero) / sizeof(uint16_t)));
}

static void system_rect_subtract_zero(sampler_rectangles_t *sample, const sampler_rectangles_t *zero)
{
    system_rect_subtract_zero_array(&sample->finger[0][0][0], &zero->finger[0][0][0],
                                    (uint16_t)(sizeof(*sample) / sizeof(uint16_t)));
}

static void system_rect_normalize_sample_array(uint16_t *sample, uint16_t count)
{
    uint16_t i;

    if (FRAME_DATA_BYTES != 1) {
        return;
    }

    for (i = 0U; i < count; ++i) {
        sample[i] = (uint16_t)(sample[i] >> 4);
    }
}

static void system_rect_normalize_sample(sampler_rectangles_t *sample)
{
    system_rect_normalize_sample_array(&sample->finger[0][0][0],
                                       (uint16_t)(sizeof(*sample) / sizeof(uint16_t)));
}

static void system_rect_apply_floor_array(uint16_t *sample, uint16_t count)
{
    uint16_t i;

    for (i = 0U; i < count; ++i) {
        if (sample[i] < 10U) {
            sample[i] = 0U;
        }
    }
}

static void system_rect_apply_floor(sampler_rectangles_t *sample)
{
    system_rect_apply_floor_array(&sample->finger[0][0][0],
                                  (uint16_t)(sizeof(*sample) / sizeof(uint16_t)));
}

#if RECT_REALTIME_FILTER_ENABLED
static void system_rect_filter(const sampler_rectangles_t *sample, sampler_rectangles_t *filtered)
{
    uint8_t finger;
    uint8_t x;
    uint8_t y;

    for (finger = 0U; finger < SAMPLER_FINGER_COUNT; ++finger) {
        uint8_t rect_id = (uint8_t)(finger + SAMPLER_RECT_THUMB);
        bilateral_filter_u16_rect(&sample->finger[finger][0][0],
                                  &filtered->finger[finger][0][0],
                                  SAMPLER_Y_COUNT, SAMPLER_X_COUNT, SAMPLER_X_COUNT);
        for (y = 0U; y < SAMPLER_Y_COUNT; ++y) {
            for (x = 0U; x < SAMPLER_X_COUNT; ++x) {
                if (sampler_rect_skip_point(rect_id, x, y) != 0U) {
                    filtered->finger[finger][y][x] = 0U;
                }
            }
        }
    }
}
#endif

static uint16_t system_rect_output_limit(void)
{
    if (FRAME_DATA_BYTES == 1U) {
        return 255U;
    }
    return 65535U;
}

static uint8_t system_rect_part_id(uint8_t rect_id)
{
    return (uint8_t)((rect_id << 4U) | 0x01U);
}

static void system_rect_clear_view(const sampler_rect_view_t *view)
{
    uint8_t x;
    uint8_t y;
    uint16_t *data;

    if ((view == NULL) || (view->data == NULL)) {
        return;
    }

    data = (uint16_t *)view->data;
    for (y = 0U; y < view->rows; ++y) {
        for (x = 0U; x < view->cols; ++x) {
            data[((uint16_t)y * view->stride) + x] = 0U;
        }
    }
}

static void system_rect_apply_force_lut_view(const sampler_rect_view_t *view)
{
    uint8_t  x;
    uint8_t  y;
    uint16_t *data;
    uint32_t sum = 0UL;
    float    predicted_force;
    uint16_t output_limit;
    uint8_t  part_id;

    if ((view == NULL) || (view->data == NULL)) {
        return;
    }

    part_id = system_rect_part_id(view->id);
    if (force_lut_is_part_loaded(part_id) == 0) {
        system_rect_clear_view(view);
        return;
    }

    data = (uint16_t *)view->data;
    for (y = 0U; y < view->rows; ++y) {
        for (x = 0U; x < view->cols; ++x) {
            sum += data[((uint16_t)y * view->stride) + x];
        }
    }

    if (sum == 0UL) {
        system_rect_clear_view(view);
        return;
    }

    predicted_force = force_lut_predict_pressure_for_part_from_u16(part_id, data,
                                                                   view->rows, view->cols);
    if (predicted_force < 0.0f) {
        system_rect_clear_view(view);
        return;
    }

    output_limit = system_rect_output_limit();
    for (y = 0U; y < view->rows; ++y) {
        for (x = 0U; x < view->cols; ++x) {
            uint16_t index = ((uint16_t)y * view->stride) + x;
            float pressure = ((float)data[index] * predicted_force) / (float)sum;

            if (pressure <= 0.0f) {
                data[index] = 0U;
            } else if (pressure >= (float)output_limit) {
                data[index] = output_limit;
            } else {
                data[index] = (uint16_t)(pressure + 0.5f);
            }
        }
    }
}

static void system_rect_apply_force_lut(sampler_rectangles_t *rects)
{
    uint8_t rect_id;

    if ((!press_flag) || (rects == NULL)) {
        return;
    }

    for (rect_id = 1U; rect_id <= SAMPLER_RECT_COUNT; ++rect_id) {
        sampler_rect_view_t view;

        if (sampler_get_rect_view(rects, rect_id, &view) != 0U) {
            system_rect_apply_force_lut_view(&view);
        }
    }
}

static void system_rect_queue_clear(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_rect_tx_queue_head = 0U;
    g_rect_tx_queue_tail = 0U;
    g_rect_tx_queue_count = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

static uint8_t system_rect_queue_free_count(void)
{
    uint8_t free_count;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    free_count = (uint8_t)(RECT_TX_QUEUE_CAPACITY - g_rect_tx_queue_count);
    if (primask == 0U) {
        __enable_irq();
    }

    return free_count;
}

static uint8_t system_rect_queue_push(uint8_t ch, uint8_t flags, const uint8_t *payload, uint16_t length)
{
    uint8_t tail;
    uint32_t primask;

    if ((payload == NULL) || (length == 0U) || (length > SAMPLER_RECT_MAX_PAYLOAD_SIZE)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (g_rect_tx_queue_count >= RECT_TX_QUEUE_CAPACITY) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }

    tail = g_rect_tx_queue_tail;
    g_rect_tx_queue[tail].channel = ch;
    g_rect_tx_queue[tail].flags = flags;
    g_rect_tx_queue[tail].length = length;
    memcpy(g_rect_tx_queue[tail].payload, payload, length);
    g_rect_tx_queue_tail = (uint8_t)((tail + 1U) % RECT_TX_QUEUE_CAPACITY);
    g_rect_tx_queue_count++;
    if (primask == 0U) {
        __enable_irq();
    }

    return 1U;
}

static uint8_t system_rect_queue_pop(rect_tx_packet_t *packet)
{
    uint8_t head;
    uint32_t primask;

    if (packet == NULL) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (g_rect_tx_queue_count == 0U) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }

    head = g_rect_tx_queue_head;
    memcpy(packet, &g_rect_tx_queue[head], sizeof(*packet));
    g_rect_tx_queue_head = (uint8_t)((head + 1U) % RECT_TX_QUEUE_CAPACITY);
    g_rect_tx_queue_count--;
    if (primask == 0U) {
        __enable_irq();
    }

    return 1U;
}

static void system_rect_tx_kick(void)
{
    rect_tx_packet_t packet;

    if ((serial1.txmtx != 0U) || (g_rect_tx_paused != 0U)) {
        return;
    }

    if (system_rect_queue_pop(&packet) == 0U) {
        return;
    }

    serial_async_send(&serial1, packet.channel, packet.flags, packet.payload, packet.length);
    if (serial1.txmtx != 0U) {
        g_debug_rect_tx_active = 1U;
    }
}

static void system_debug_rect_tx_complete(void)
{
    uint32_t now;
    uint32_t elapsed;

    g_debug_rect_tx_active = 0U;
    ++g_debug_rect_tx_packets;
    ++g_debug_rect_packets_in_hand;

    if (g_debug_rect_packets_in_hand < RECT_SEND_COUNT) {
        return;
    }

    g_debug_rect_packets_in_hand = 0U;
    ++g_debug_hand_tx_count;
    ++g_debug_hands_in_window;

    now = (g_system_port.get_tick != NULL) ? g_system_port.get_tick() : 0U;
    elapsed = now - g_debug_hand_window_start_ms;
    if (elapsed >= 1000U) {
        g_debug_hand_fps_milli = (g_debug_hands_in_window * 1000000UL) / elapsed;
        g_debug_hand_window_ms = elapsed;
        g_debug_hands_in_window = 0U;
        g_debug_hand_window_start_ms = now;
    }
}

static void system_queue_next_rect_frame(void)
{
    sampler_rectangles_t *send_rects;
    sampler_rect_view_t view;
    uint16_t payload_size;
    uint8_t rect_index;
    uint8_t output_id;

    if (system_rect_queue_free_count() < RECT_SEND_COUNT) {
        return;
    }

    sampler_capture_rectangles(&g_sampler_rects);
    system_rect_normalize_sample(&g_sampler_rects);
    system_rect_subtract_zero(&g_sampler_rects, &g_sampler_zero_rects);

#if RECT_REALTIME_FILTER_ENABLED
    if (config_.filter_enabled) {
        system_rect_filter(&g_sampler_rects, &g_sampler_filter_rects);
        system_rect_apply_floor(&g_sampler_filter_rects);
        send_rects = &g_sampler_filter_rects;
    } else {
#endif
        system_rect_apply_floor(&g_sampler_rects);
        send_rects = &g_sampler_rects;
#if RECT_REALTIME_FILTER_ENABLED
    }
#endif

    if (press_flag) {
        system_rect_apply_force_lut(send_rects);
    }

    for (rect_index = 0U; rect_index < RECT_SEND_COUNT; ++rect_index) {
        if (sampler_get_rect_view(send_rects, g_rect_send_order[rect_index], &view) == 0U) {
            return;
        }

        payload_size = sampler_pack_finger_payload(send_rects, view.id,
                                                   g_rect_payload, sizeof(g_rect_payload));
        if (payload_size == 0U) {
            return;
        }

        output_id = RECT_OUTPUT_ID(view.id);
        if (system_rect_queue_push((uint8_t)((output_id << 4U) | 0x02U), 0x00U,
                                   g_rect_payload, payload_size) == 0U) {
            return;
        }
    }
}

static void system_capture_rect_zero(void)
{
    uint16_t frame;
    uint16_t frames = config_.zeroing.frames;

    if (g_rect_zero_ready != 0U) {
        return;
    }

    if (!config_.zeroing.enabled) {
        config_.zeroing.finished = true;
        g_rect_zero_ready = 1U;
        return;
    }

    if (frames == 0U) {
        frames = 1U;
    }

    memset(&g_sampler_zero_rects, 0, sizeof(g_sampler_zero_rects));
    for (frame = 0U; frame < frames; ++frame) {
        sampler_capture_rectangles(&g_sampler_rects);
        system_rect_accumulate_max(&g_sampler_zero_rects, &g_sampler_rects);
    }
    system_rect_apply_zero(&g_sampler_zero_rects);
    config_.zeroing.finished = true;
    g_rect_zero_ready = 1U;
    system_rect_queue_clear();
}

static void system_service_rect_tx(void)
{
    if (g_rect_zero_ready == 0U) {
        system_capture_rect_zero();
        return;
    }

    system_queue_next_rect_frame();
    system_rect_tx_kick();
}

static void serial_handle_rx(uint8_t from_isr)
{
    if (serial1.flag == 0U) {
        return;
    }

    serial1.flag = 0U;
    g_serial_dispatch_from_isr = from_isr;
    unpack(&serial1);
    g_serial_dispatch_from_isr = 0U;
}

static void serial_dispatch_cmd(package_t *package)
{
    if (serial_cmd_requires_tx_pause(package) != 0U) {
        g_rect_tx_paused = 1U;
        g_rect_tx_resume_after_tx = 0U;
        if ((g_serial_dispatch_from_isr != 0U) || (serial1.txmtx != 0U)) {
            memcpy(&g_serial_deferred_package, package, sizeof(g_serial_deferred_package));
            g_serial_deferred_pending = 1U;
            return;
        }

        config_execute_cmd(package);
        if (serial1.txmtx != 0U) {
            g_rect_tx_resume_after_tx = 1U;
        } else {
            g_rect_tx_paused = 0U;
            system_rect_tx_kick();
        }
        return;
    }

    if ((g_serial_dispatch_from_isr != 0U) && (serial_cmd_is_irq_safe(package) == 0U)) {
        memcpy(&g_serial_deferred_package, package, sizeof(g_serial_deferred_package));
        g_serial_deferred_pending = 1U;
        return;
    }

    config_execute_cmd(package);
}

static void serial_handle_deferred_cmd(void)
{
    if (g_serial_deferred_pending == 0U) {
        return;
    }

    g_serial_deferred_pending = 0U;
    if ((g_rect_tx_paused != 0U) && (serial_cmd_requires_tx_pause(&g_serial_deferred_package) != 0U)) {
        if (serial1.txmtx != 0U) {
            g_serial_deferred_pending = 1U;
            return;
        }

        config_execute_cmd(&g_serial_deferred_package);
        if (serial1.txmtx != 0U) {
            g_rect_tx_resume_after_tx = 1U;
        } else {
            g_rect_tx_paused = 0U;
            system_rect_tx_kick();
        }
        return;
    }

    config_execute_cmd(&g_serial_deferred_package);
}

static uint8_t serial_cmd_is_irq_safe(const package_t *package)
{
    uint8_t channel = package->channel & 0x0f;
    uint8_t cmd;

    if (channel == 0x02U) {
        return 1U;
    }

    if ((channel != 0x01U) || (package->length == 0U) || (serial1.txmtx != 0U)) {
        return 0U;
    }

    cmd = package->payload[0];
    switch (cmd) {
    case 0x01U:
    case 0x02U:
    case 0x03U:
    case 0x05U:
    case 0x06U:
        return 1U;
    default:
        return 0U;
    }
}

// ======================= 压力标定处理模块 =======================
static void pressure_process_cal_area(uint8_t area_num, 
                                     uint8_t start_col, uint8_t end_col,
                                     uint8_t start_row, uint8_t end_row,
                                     uint16_t valid_points)
{
    uint64_t ad_value_sum = 0;
    uint16_t ad_value;
    
    // 计算AD值总和
    for (int i = start_col; i <= end_col; i++) {
        for (int j = start_row; j <= end_row; j++) {
            ad_value = (pbuf[2 * (i * ROWS + j) + 1] << 8) | pbuf[2 * (i * ROWS + j)];
            ad_value_sum += ad_value;
        }
    }
    
    // 计算AD平均值
    float ad_value_avg = (float)ad_value_sum / valid_points;
    
    // 选择对应的查找表
    float (*lut)[PRESS_LUT_MAX_SIZE];
    uint8_t count;
    switch(area_num) {
        case 1:
            lut = press_lut_1;
            count = press_lut_cout_1;
            break;
        case 2:
            lut = press_lut_2;
            count = press_lut_cout_2;
            break;
        case 3:
            lut = press_lut_3;
            count = press_lut_cout_3;
            break;
        case 4:
            lut = press_lut_4;
            count = press_lut_cout_4;
            break;
        default:
            return;
    }
    
    // 查找位置并计算压力值
    int position = findPosition(lut[0], count, (uint16_t)ad_value_avg);
    float pressure = 0;
    
    if (position == -1) {
        pressure = 0;
    } else if (position == count - 1) {
        pressure = lut[1][count - 1];
    } else {
        if ((lut[0][position + 1] - lut[0][position]) > 0) {
            pressure = lut[1][position] + 
                      (ad_value_avg - lut[0][position]) * 
                      (lut[1][position + 1] - lut[1][position]) / 
                      (lut[0][position + 1] - lut[0][position]);
        } else {
            pressure = lut[1][position];
        }
    }
    
    // 限制压力值范围
    int pressure_sum;
    if (pressure < 0) {
        pressure_sum = 0;
    } else if (pressure > 65535) {
        pressure_sum = 65535;
    } else {
        pressure_sum = (int)(pressure + 0.5f);
    }
    
    // 应用压力转换
    uint8_t transformer[FRAME_SIZE] = { 0 };
    memcpy(transformer, pbuf, PACKAGE_SIZE - 4);
	
    pressure_transform_calibration(transformer, pbuf, ad_value_sum, pressure_sum,
                                  start_col, end_col, start_row, end_row);

}

static void pressure_calibration_process(void)
{
    if (!press_flag) return;
    
    // 处理四个标定区域
    pressure_process_cal_area(1, 
                             CALIBRATION_1_START_COLS, CALIBRATION_1_END_COLS,
                             CALIBRATION_1_START_ROWS, CALIBRATION_1_END_ROWS,
                             CALIBRATION_1_VALID_POINTS);
    
//    pressure_process_cal_area(2,
//                             CALIBRATION_2_START_COLS, CALIBRATION_2_END_COLS,
//                             CALIBRATION_2_START_ROWS, CALIBRATION_2_END_ROWS,
//                             CALIBRATION_2_VALID_POINTS);
//    
//    pressure_process_cal_area(3,
//                             CALIBRATION_3_START_COLS, CALIBRATION_3_END_COLS,
//                             CALIBRATION_3_START_ROWS, CALIBRATION_3_END_ROWS,
//                             CALIBRATION_3_VALID_POINTS);
//    
//    pressure_process_cal_area(4,
//                             CALIBRATION_4_START_COLS, CALIBRATION_4_END_COLS,
//                             CALIBRATION_4_START_ROWS, CALIBRATION_4_END_ROWS,
//                             CALIBRATION_4_VALID_POINTS);
}

// ======================= Flash配置读取模块 =======================
void get_pressure_cal_cfg()
{
    uint8_t press_data[PRESS_LUT_MAX_SIZE * 2U * sizeof(float)] = { 0 };
    uint8_t test_data = 0U;
    uint8_t test_flag = 0U;
    uint16_t press_data_bytes;
    
    // 标定区域1
    if (FLASH_OK == flash_read(ADDR_FLASH_SECTOR_PRESS_LUT_COUNT, &test_data, 1U)) {
        press_lut_cout_1 = test_data;
    }
    if (press_lut_cout_1 > PRESS_LUT_MAX_SIZE)
        press_lut_cout_1 = PRESS_LUT_MAX_SIZE;
    
    press_data_bytes = (uint16_t)(press_lut_cout_1 * 2U * sizeof(float));
    if ((press_data_bytes > 0U) &&
        (FLASH_OK == flash_read(ADDR_FLASH_SECTOR_PRESS_LUT_DATA, press_data, press_data_bytes))) {
        memcpy(press_lut_1[0], press_data, press_lut_cout_1 * sizeof(float));
        memcpy(press_lut_1[1], &press_data[press_lut_cout_1 * sizeof(float)],
               press_lut_cout_1 * sizeof(float));
    }
    
    // 标定区域2
#if (PRESS_CALIBRATION_AREA_COUNT > 1U)
    test_data = 0U;
    if (FLASH_OK == flash_read(ADDR_FLASH_SECTOR_36, &test_data, 1U)) {
        press_lut_cout_2 = test_data;
    }
    if (press_lut_cout_2 > PRESS_LUT_MAX_SIZE)
        press_lut_cout_2 = PRESS_LUT_MAX_SIZE;
    
    press_data_bytes = (uint16_t)(press_lut_cout_2 * 2U * sizeof(float));
    if ((press_data_bytes > 0U) &&
        (FLASH_OK == flash_read(ADDR_FLASH_SECTOR_33, press_data, press_data_bytes))) {
        memcpy(press_lut_2[0], press_data, press_lut_cout_2 * sizeof(float));
        memcpy(press_lut_2[1], &press_data[press_lut_cout_2 * sizeof(float)],
               press_lut_cout_2 * sizeof(float));
    }
    
    // 标定区域3
    test_data = 0U;
    if (FLASH_OK == flash_read(ADDR_FLASH_SECTOR_37, &test_data, 1U)) {
        press_lut_cout_3 = test_data;
    }
    if (press_lut_cout_3 > PRESS_LUT_MAX_SIZE)
        press_lut_cout_3 = PRESS_LUT_MAX_SIZE;
    
    press_data_bytes = (uint16_t)(press_lut_cout_3 * 2U * sizeof(float));
    if ((press_data_bytes > 0U) &&
        (FLASH_OK == flash_read(ADDR_FLASH_SECTOR_34, press_data, press_data_bytes))) {
        memcpy(press_lut_3[0], press_data, press_lut_cout_3 * sizeof(float));
        memcpy(press_lut_3[1], &press_data[press_lut_cout_3 * sizeof(float)],
               press_lut_cout_3 * sizeof(float));
    }
    
    // 标定区域4
    test_data = 0U;
    if (FLASH_OK == flash_read(ADDR_FLASH_SECTOR_38, &test_data, 1U)) {
        press_lut_cout_4 = test_data;
    }
    if (press_lut_cout_4 > PRESS_LUT_MAX_SIZE)
        press_lut_cout_4 = PRESS_LUT_MAX_SIZE;
    
    press_data_bytes = (uint16_t)(press_lut_cout_4 * 2U * sizeof(float));
    if ((press_data_bytes > 0U) &&
        (FLASH_OK == flash_read(ADDR_FLASH_SECTOR_35, press_data, press_data_bytes))) {
        memcpy(press_lut_4[0], press_data, press_lut_cout_4 * sizeof(float));
        memcpy(press_lut_4[1], &press_data[press_lut_cout_4 * sizeof(float)],
               press_lut_cout_4 * sizeof(float));
    }
    
    // 读取压力标志
#endif
    if (FLASH_OK == flash_read(ADDR_FLASH_SECTOR_PRESS_MODE_FLAG, &test_flag, 1U)) {
        press_flag = (test_flag == 0x01U);
    }

    (void)force_lut_flash_load_all();
}

// ======================= 系统主模块 =======================
void system_init()
{
    // 初始化串口
    serial_init();
    
    // 设置缓冲区指针
    pbuf = package + 4;
    size = PACKAGE_SIZE;
    g_upload_cache_active = 0U;
    g_upload_cache_ready = 0U;
    g_upload_cache_size = 0U;
    g_debug_rect_tx_packets = 0U;
    g_debug_hand_tx_count = 0U;
    g_debug_hand_fps_milli = 0U;
    g_debug_hand_window_ms = 0U;
    g_debug_rect_tx_active = 0U;
    g_debug_rect_packets_in_hand = 0U;
    g_debug_hands_in_window = 0U;
    g_debug_hand_window_start_ms =
        (g_system_port.get_tick != NULL) ? g_system_port.get_tick() : 0U;
    
    // 初始化硬件
    if (g_system_port.hardware_init != NULL) {
        g_system_port.hardware_init();
    }
    
    // 读取Flash配置
    flash_init();
    get_pressure_cal_cfg();
    flash_deinit();
    
    // 读取系统配置
    uint32_t config_flash_addr = ADDR_FLASH_SECTOR_CONFIG;
    if (!config_read(&config_, config_flash_addr)) {
        uint32_t u32Primask;
        config_init_default(&config_);

        u32Primask = __get_PRIMASK();
        __disable_irq();
        if (FLASH_OK == flash_init()) {
            (void)config_write(&config_, config_flash_addr);
            (void)flash_deinit();
        }
        if (0U == u32Primask) {
            __enable_irq();
        }
    }
    serial_toggle_tx(!config_.multiple_serial_enabled);
    serial1.address = config_.address;
    config_.zeroing.finished = false;
    g_rect_zero_ready = 0U;
    system_rect_queue_clear();
    g_rect_tx_paused = 0U;
    g_rect_tx_resume_after_tx = 0U;
}

static uint8_t serial_cmd_requires_tx_pause(const package_t *package)
{
    uint8_t channel;

    if (package == NULL) {
        return 0U;
    }

    channel = package->channel & 0x0f;
    switch (channel) {
    case 0x01U:
    case 0x06U:
    case 0x07U:
    case 0x09U:
    case 0x0eU:
    case 0x0fU:
        return 1U;
    default:
        return 0U;
    }
}

void system_set_port(const system_port_t *port)
{
    if (port == NULL) {
        memset(&g_system_port, 0, sizeof(g_system_port));
        return;
    }

    g_system_port = *port;
}

uint8_t *system_get_serial_rx_buffer(void)
{
    return serial1.rxbuf;
}

uint16_t system_get_serial_rx_buffer_size(void)
{
    return SERIAL_RX_BUFFER_SIZE;
}

uint8_t *system_get_upload_buffer(void)
{
    if (g_upload_cache_ready == 0U) {
        return NULL;
    }

    return g_upload_cache[g_upload_cache_active];
}

uint16_t system_get_upload_size(void)
{
    return g_upload_cache_size;
}

void system_rect_zero_clear(void)
{
    memset(&g_sampler_zero_rects, 0, sizeof(g_sampler_zero_rects));
    system_rect_queue_clear();
    g_rect_zero_ready = 0U;
    g_rect_tx_paused = 0U;
    g_rect_tx_resume_after_tx = 0U;
    config_.zeroing.finished = false;
}

void system_set_upload_paused(uint8_t paused)
{
    g_rect_tx_paused = (paused != 0U) ? 1U : 0U;
    if (g_rect_tx_paused == 0U) {
        system_rect_tx_kick();
    }
}

void system_serial_rx_notify(uint16_t rx_size)
{
    serial1.rxsize = rx_size;
    if (rx_size != 0U) {
        serial1.flag = 1U;
        serial_handle_rx(1U);
    }
}

void system_serial_tx_complete(void)
{
    if (g_debug_rect_tx_active != 0U) {
        system_debug_rect_tx_complete();
    }

    if (serial1.txmtx != 0U) {
        serial_tx_unlock(&serial1);
    }

    if (g_rect_tx_resume_after_tx != 0U) {
        g_rect_tx_resume_after_tx = 0U;
        g_rect_tx_paused = 0U;
    }

    system_rect_tx_kick();
}

void system_run()
{
    serial_handle_deferred_cmd();
    serial_handle_rx(0U);

    system_service_rect_tx();

    serial_handle_deferred_cmd();
    serial_handle_rx(0U);
    return;

    // 处理串口接收
    serial_handle_deferred_cmd();
    serial_handle_rx(0U);
    
    // 校准处理
    calibration_feed(&config_);
    
    // 获取并处理帧数据
    if (config_.filter_enabled) {
        uint8_t frame[FRAME_SIZE] = { 0 };
        sampler_get_frame(frame);
        calibration_zero(&config_, frame);
        bilateral_filter(frame, pbuf);
    } else {
        sampler_get_frame(pbuf);
        calibration_zero(&config_, pbuf);
    }
    
    // 压力标定处理
    if (press_flag) {
        pressure_calibration_process();
    }
    
    // 压力总和检查
    uint32_t z_pressure = 0;
    for (uint16_t i = 0U; i < (uint16_t)(COLS * ROWS); ++i) {
        uint16_t val = (pbuf[2 * i + 1] << 8) | pbuf[2 * i];
        z_pressure += val;
    }
    
    if (z_pressure < 20) {
        memset(package + 4, 0, FRAME_SIZE);
    }
    
    // 数据处理
    //data_handle();
    system_publish_upload_frame(package, size);
    
    // 数据发送
    if (config_.multiple_serial_enabled) {
        if (config_.zeroing.finished && package_id != 0xff) {
            if (serial1.txmtx == 0U) {
                uint8_t *upload_buffer = system_get_upload_buffer();
                uint16_t upload_size = system_get_upload_size();

                if ((upload_buffer != NULL) && (upload_size != 0U)) {
                    serial_async_send(&serial1, (config_.address << 4) | 0x02,
                                      package_id | 0x03, upload_buffer, upload_size);
                    package_id = 0xff;
                }
            }
        }
    } else {
        if (config_.zeroing.finished) {
            if (serial1.txmtx == 0U) {
                uint8_t *upload_buffer = system_get_upload_buffer();
                uint16_t upload_size = system_get_upload_size();

                if ((upload_buffer != NULL) && (upload_size != 0U)) {
                    serial_async_send(&serial1, 0x02, 0X00, upload_buffer, upload_size);
                    package_id = 0xff;
                }
            }
        }
    }

    serial_handle_deferred_cmd();
    serial_handle_rx(0U);
}

// ======================= 查找位置函数（保持原有） =======================
static int findPosition(float arr[], uint8_t size, uint16_t target) 
{
    if (size == 0) return -1;
    if (target < arr[0]) return -1; 
    
    uint8_t left = 0;
    uint8_t right = size - 1;
    
    while (left <= right) {
        int mid = left + (right - left) / 2;
        
        if (arr[mid] == target) {
            return mid; 
        } else if (arr[mid] < target) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    
    return right;
}

static void serial_toggle_tx(uint8_t en)
{
    if (g_system_port.toggle_tx != NULL) {
        g_system_port.toggle_tx(en);
    }
}
