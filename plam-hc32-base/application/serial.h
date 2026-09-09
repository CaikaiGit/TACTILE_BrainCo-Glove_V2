#ifndef APPLICATION_SERIAL_H
#define APPLICATION_SERIAL_H

#include <stddef.h>
#include <stdint.h>

#define SERIAL_RX_BUFFER_SIZE 2048
#define SERIAL_TX_BUFFER_SIZE 768

typedef struct
{
    uint8_t  channel;
    uint8_t  flags;
    uint16_t length;
    uint8_t  payload[SERIAL_RX_BUFFER_SIZE];
    uint16_t checksum;

    // unpack
    uint8_t state;

    // RW
    uint16_t widx;
    uint16_t ridx;
} package_t;

typedef void(SerialOnReadyHandler)(package_t *package);
typedef void(SerialOnErrorHandler)(uint8_t /*error*/, const char * /*message*/);

typedef void(SerialToggleTxCallback)(uint8_t en);
typedef uint32_t(SerialGetTickCallback)(void);
typedef void(SerialDelayCallback)(uint32_t ms);
typedef int32_t(SerialWriteCallback)(void *ctx, const uint8_t *data, uint16_t size);

typedef struct
{
    void *uart;
    void *instance;
    void *dma_usart_rx;
    void *write_ctx;
    
    uint8_t                 address;
    SerialToggleTxCallback *toggle_tx;
    SerialGetTickCallback  *get_tick;
    SerialDelayCallback    *delay_ms;
    SerialWriteCallback    *write;
    SerialWriteCallback    *write_dma;

    // tx
    uint8_t volatile /* MUST */ txmtx;
    uint16_t txsize;
    uint8_t  txbuf[SERIAL_TX_BUFFER_SIZE];

    // rx
    uint8_t volatile rxmtx;
    uint16_t              rxsize;
    uint8_t               rxbuf[SERIAL_RX_BUFFER_SIZE];
    SerialOnReadyHandler *onready;
    SerialOnErrorHandler *onerror;

    uint8_t volatile flag;
    // unpacker
    package_t package;
} serial_t;

/// package

uint16_t package_remain_size(package_t *package);

void package_read(package_t *, void *data, size_t size);

uint8_t package_read_u8(package_t *);

uint16_t package_read_u16(package_t *);

uint32_t package_read_u32(package_t *);

float package_read_f32(package_t *);

void     package_skip(package_t *, size_t size);
uint16_t pack(uint8_t *buffer, uint8_t ch, uint8_t flags, void *payload, uint16_t len);

///

void serial_tx_lock(serial_t *serial);

void serial_tx_unlock(serial_t *serial);

/// TX

void serial_async_send(serial_t *serial, uint8_t ch, uint8_t flags, void *payload, uint16_t size);

void serial_send(serial_t *serial, uint8_t ch, uint8_t flags, void *payload, uint16_t size);

/// RX

void unpack(serial_t *serial);

///

uint16_t crc_16(const uint8_t *data, size_t length);

#endif // APPLICATION_SERIAL_H
