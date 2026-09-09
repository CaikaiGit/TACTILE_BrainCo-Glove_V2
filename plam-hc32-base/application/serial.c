#include "serial.h"

#include "configuration.h"
#include "hc32f4xx.h"

#include <stdint.h>
#include <string.h>
extern config_t config_;

static uint32_t serial_now(serial_t *serial)
{
    return (serial->get_tick != NULL) ? serial->get_tick() : 0U;
}

static void serial_wait_ms(serial_t *serial, uint32_t ms)
{
    if (serial->delay_ms != NULL) {
        serial->delay_ms(ms);
    }
}

static int serial_is_in_isr(void)
{
    return (__get_IPSR() != 0U);
}

static int serial_tx_try_lock(serial_t *serial, uint8_t wait)
{
    if (wait == 0U) {
        if (serial->txmtx != 0U) {
            return 0;
        }
    } else {
        uint32_t start_tick = serial_now(serial);

        while (serial->txmtx != 0U) {
            if ((serial_now(serial) - start_tick) > 50U) {
                serial->txmtx = 0U;
                return 0;
            }
            serial_wait_ms(serial, 1U);
        }
    }

    if (config_.multiple_serial_enabled) {
        if (serial->address != 0x00 && serial->address != 0xff && serial->toggle_tx) {
            serial->toggle_tx(0x01);
        }
    }
    else if (serial->toggle_tx != NULL) {
        serial->toggle_tx(0x01);
    }

    serial->txmtx = 1U;
    return 1;
}

uint16_t package_remain_size(package_t *package) { return package->length - package->ridx; }

void  package_read(package_t *package, void *data, size_t size)
{
    uint16_t remain_size = package_remain_size(package);
    size_t   bytes       = (size > remain_size) ? remain_size : size;

    memcpy(data, package->payload + package->ridx, bytes);

     package_skip(package, bytes);
}

uint8_t package_read_u8(package_t *package)
{
    uint8_t value = 0;
     package_read(package, &value, 1);

    return value;
}

uint16_t package_read_u16(package_t *package)
{
    uint16_t value = 0;
    package_read(package, &value, 2);

    return value;
}

uint32_t package_read_u32(package_t *package)
{
    uint32_t value = 0;
    package_read(package, &value, 4);

    return value;
}

float package_read_f32(package_t *package)
{
    float value = 0;
    package_read(package, &value, 4);

    return value;
}

void package_skip(package_t *package, size_t size)
{
    package->ridx += size;
    if (package->ridx > package->length) package->ridx = package->length;
}

///

void serial_tx_lock(serial_t *serial)
{
    (void)serial_tx_try_lock(serial, 1U);
}

void serial_tx_unlock(serial_t *serial)
{
    serial->txmtx = 0;
    if (config_.multiple_serial_enabled) {
        if (serial->address != 0x00 && serial->address != 0xff && serial->toggle_tx)
            serial->toggle_tx(0x00);
    }
}

///

uint16_t pack(uint8_t *buffer, uint8_t ch, uint8_t flags, void *payload, uint16_t len)
{
    // head
    buffer[0] = '<';
    buffer[1] = '<';

    // channel
    buffer[2] = ch;

    // flags
    buffer[3] = flags;

    // length
    buffer[4] = (uint8_t)(len & 0xff);
    buffer[5] = (uint8_t)(len >> 8);

    // payload
    memcpy(buffer + 6, payload, len);

    uint16_t checksum = crc_16(payload, len);

    // checksum
    buffer[6 + len + 0] = (uint8_t)(checksum & 0xff);
    buffer[6 + len + 1] = (uint8_t)(checksum >> 8);

    buffer[6 + len + 2] = '>';
    buffer[6 + len + 3] = '>';

    return len + 10;
}

/// TX

void serial_async_send(serial_t *serial, uint8_t ch, uint8_t flags, void *payload, uint16_t length)
{
    uint8_t  buffer[SERIAL_TX_BUFFER_SIZE];
    uint16_t size = pack(buffer, ch, flags, payload, length);
    if (!serial_tx_try_lock(serial, (uint8_t)!serial_is_in_isr())) {
        return;
    }
    serial->txsize = size;
    memcpy(serial->txbuf, buffer, serial->txsize);
    if (serial->write_dma != NULL) {
        if (serial->write_dma(serial->write_ctx, serial->txbuf, serial->txsize) < 0) {
            serial_tx_unlock(serial);
        }
    } else {
        serial_tx_unlock(serial);
    }
}

void serial_send(serial_t *serial, uint8_t ch, uint8_t flags, void *data, uint16_t size)
{
    uint8_t pack[10] = {
        '<', '<', ch, flags, (uint8_t)(size & 0xff), (uint8_t)(size >> 8), 0x00, 0x00, '>', '>',
    };

    uint16_t checksum = crc_16(data, size);

    // checksum
    pack[6] = (uint8_t)(checksum & 0xff);
    pack[7] = (uint8_t)(checksum >> 8);

    serial_tx_lock(serial);

    if (serial->write != NULL) {
        (void)serial->write(serial->write_ctx, pack, 6);
        (void)serial->write(serial->write_ctx, data, size);
        (void)serial->write(serial->write_ctx, pack + 6, 4);
    }

    serial_tx_unlock(serial);
}

/// RX

enum
{
    S_HEAD_L     = 0,
    S_HEAD_H     = 1,
    S_CHANNEL    = 2,
    S_FLAGS      = 3,
    S_LENGTH_L   = 4,
    S_LENGTH_H   = 5,
    S_PAYLOAD    = 6,
    S_CHECKSUM_L = 7,
    S_CHECKSUM_H = 8,
    S_TAIL_L     = 9,
    S_TAIL_H     = 10
};

void unpack(serial_t *serial)
{
    for (size_t i = 0; i < serial->rxsize; ++i) {
        uint8_t ch = serial->rxbuf[i];

        switch (serial->package.state) {
        case S_HEAD_L: serial->package.state = (ch == '<') ? S_HEAD_H : S_HEAD_L; break;
        case S_HEAD_H: serial->package.state = (ch == '<') ? S_CHANNEL : S_HEAD_L; break;

        case S_CHANNEL:
            serial->package.channel = ch;
            serial->package.state   = S_FLAGS;
            break;

        case S_FLAGS:
            serial->package.flags = ch;
            serial->package.state = S_LENGTH_L;
            break;

        case S_LENGTH_L:
            serial->package.length = ch;
            serial->package.state  = S_LENGTH_H;
            break;
        case S_LENGTH_H:
            serial->package.length = (ch << 8) | serial->package.length;
            serial->package.state  = S_PAYLOAD;
            serial->package.widx   = 0;
            serial->package.ridx   = 0;
            break;

        case S_PAYLOAD:
            if (serial->package.widx < serial->package.length) {
                serial->package.payload[serial->package.widx++] = ch;
            }

            if (serial->package.widx == serial->package.length) serial->package.state = S_CHECKSUM_L;
            break;

        case S_CHECKSUM_L:
            serial->package.checksum = ch;
            serial->package.state    = S_CHECKSUM_H;
            break;
        case S_CHECKSUM_H:
            serial->package.checksum = (ch << 8) | serial->package.checksum;
            serial->package.state    = S_TAIL_L;
            break;

        case S_TAIL_L: serial->package.state = (ch == '>') ? S_TAIL_H : S_HEAD_L; break;

        case S_TAIL_H:
            serial->package.state = S_HEAD_L;

            if (ch == '>' &&
                (serial->package.checksum == crc_16(serial->package.payload, serial->package.length))) {

                if (serial->onready) serial->onready(&(serial->package));
                break;
            }
            // if (ch == '>') {

            //     if (serial->onready) serial->onready(&(serial->package));
            //     break;
            // }
            else if (serial->onerror) {
                //                serial->onerror(0x01, "[UNPACK] CHECKSUM ERROR");
            }
            break;
        default: break;
        }
    }

    serial->rxsize = 0;
}

///

uint16_t crc_16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0;

    while (length--) {
        crc ^= *data++;

        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}
