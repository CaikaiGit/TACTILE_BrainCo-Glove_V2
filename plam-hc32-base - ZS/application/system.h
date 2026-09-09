#ifndef __SYSTEM_H
#define __SYSTEM_H

#include <stdint.h>

#define PRESS_LUT_MAX_SIZE (62U)

typedef uint32_t(SystemGetTickCallback)(void);
typedef void(SystemDelayMsCallback)(uint32_t ms);
typedef int32_t(SystemSerialWriteCallback)(void *ctx, const uint8_t *data, uint16_t size);
typedef void(SystemSerialToggleTxCallback)(uint8_t en);
typedef void(SystemSerialStartRxCallback)(void *ctx, uint8_t *buffer, uint16_t size);
typedef void(SystemHardwareInitCallback)(void);

typedef struct
{
    void *serial_ctx;
    SystemGetTickCallback *get_tick;
    SystemDelayMsCallback *delay_ms;
    SystemSerialWriteCallback *write;
    SystemSerialWriteCallback *write_dma;
    SystemSerialToggleTxCallback *toggle_tx;
    SystemSerialStartRxCallback *start_rx;
    SystemHardwareInitCallback *hardware_init;
} system_port_t;

void system_set_port(const system_port_t *port);
uint8_t *system_get_serial_rx_buffer(void);
uint16_t system_get_serial_rx_buffer_size(void);
uint8_t *system_get_upload_buffer(void);
uint16_t system_get_upload_size(void);
void system_rect_zero_clear(void);
void system_set_upload_paused(uint8_t paused);
void get_pressure_cal_cfg(void);
void system_serial_tx_complete(void);
void system_init(void);
void system_run(void);
void system_serial_rx_notify(uint16_t rx_size);
void system_prepare_for_reset(void);

#endif
