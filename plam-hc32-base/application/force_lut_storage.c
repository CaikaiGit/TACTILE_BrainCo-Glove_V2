#include "force_lut_storage.h"

#include "internal_flash.h"
#include "serial.h"

#include <string.h>

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t image_size;
    uint16_t crc;
    uint16_t reserved;
    force_lut_part_data_t part;
} force_lut_flash_image_t;

typedef char force_lut_image_size_check[
    (sizeof(force_lut_flash_image_t) <= FORCE_LUT_FLASH_SLOT_SIZE) ? 1 : -1];

static uint8_t g_force_lut_write_buf[FORCE_LUT_FLASH_SLOT_SIZE];

static uint32_t force_lut_flash_addr_by_index(uint8_t index)
{
    switch (index) {
    case 0U: return ADDR_FLASH_SECTOR_FORCE_LUT_PART1;
    case 1U: return ADDR_FLASH_SECTOR_FORCE_LUT_PART2;
    case 2U: return ADDR_FLASH_SECTOR_FORCE_LUT_PART3;
    case 3U: return ADDR_FLASH_SECTOR_FORCE_LUT_PART4;
    case 4U: return ADDR_FLASH_SECTOR_FORCE_LUT_PART5;
    case 5U: return ADDR_FLASH_SECTOR_FORCE_LUT_PART6;
    default: return 0UL;
    }
}

static uint8_t force_lut_part_id_by_index(uint8_t index)
{
    return (uint8_t)(((index + 1U) << 4U) | 0x01U);
}

int force_lut_part_index(uint8_t part_id)
{
    switch (part_id) {
    case 0x11U: return 0;
    case 0x21U: return 1;
    case 0x31U: return 2;
    case 0x41U: return 3;
    case 0x51U: return 4;
    case 0x61U: return 5;
    default: return -1;
    }
}

static uint8_t force_lut_axis_increasing(const float *axis, uint16_t count)
{
    uint16_t i;

    if ((axis == 0) || (count == 0U)) {
        return 0U;
    }

    for (i = 1U; i < count; ++i) {
        if (axis[i] <= axis[i - 1U]) {
            return 0U;
        }
    }

    return 1U;
}

int force_lut_part_data_valid(const force_lut_part_data_t *part)
{
    if (part == 0) {
        return 0;
    }
    if (force_lut_part_index(part->part_id) < 0) {
        return 0;
    }
    if ((part->sensor_rows == 0U) || (part->sensor_cols == 0U) ||
        (part->area_num == 0U) || (part->area_num > FORCE_LUT_MAX_AREA_NUM) ||
        (part->force_num == 0U) || (part->force_num > FORCE_LUT_MAX_FORCE_NUM)) {
        return 0;
    }
    if (((uint32_t)part->area_num * (uint32_t)part->force_num) >
        (uint32_t)FORCE_LUT_MAX_TABLE_COUNT) {
        return 0;
    }

    return force_lut_axis_increasing(part->force_grid, part->force_num) != 0U;
}

int force_lut_apply_part_data(const force_lut_part_data_t *part)
{
    if (force_lut_part_data_valid(part) == 0) {
        return FORCE_LUT_ERR_DIMENSION;
    }

    return force_lut_load_area_table_for_part_ex(part->part_id,
                                                part->sensor_rows,
                                                part->sensor_cols,
                                                part->area_num,
                                                part->force_num,
                                                part->area_cx,
                                                part->area_cy,
                                                part->force_grid,
                                                part->sensor_table);
}

int force_lut_flash_write_part(const force_lut_part_data_t *part)
{
    force_lut_flash_image_t image;
    uint32_t                addr;
    int                     index;

    if (force_lut_part_data_valid(part) == 0) {
        return 0;
    }

    index = force_lut_part_index(part->part_id);
    if (index < 0) {
        return 0;
    }

    addr = force_lut_flash_addr_by_index((uint8_t)index);
    if (addr == 0UL) {
        return 0;
    }

    memset(&image, 0, sizeof(image));
    image.magic      = FORCE_LUT_FLASH_MAGIC;
    image.version    = FORCE_LUT_FLASH_VERSION;
    image.image_size = (uint16_t)sizeof(image);
    image.part       = *part;
    image.crc        = crc_16((const uint8_t *)&image.part, sizeof(image.part));

    memset(g_force_lut_write_buf, 0xFF, sizeof(g_force_lut_write_buf));
    memcpy(g_force_lut_write_buf, &image, sizeof(image));

    return (flash_pack_and_write(addr, g_force_lut_write_buf, sizeof(g_force_lut_write_buf)) == FLASH_OK) ? 1 : 0;
}

int force_lut_flash_load_part(uint8_t part_id)
{
    force_lut_flash_image_t image;
    uint32_t                addr;
    int                     index;

    index = force_lut_part_index(part_id);
    if (index < 0) {
        return 0;
    }

    addr = force_lut_flash_addr_by_index((uint8_t)index);
    if (addr == 0UL) {
        return 0;
    }

    memset(&image, 0, sizeof(image));
    if (flash_read(addr, &image, (uint16_t)sizeof(image)) != FLASH_OK) {
        return 0;
    }
    if ((image.magic != FORCE_LUT_FLASH_MAGIC) ||
        (image.version != FORCE_LUT_FLASH_VERSION) ||
        (image.image_size != (uint16_t)sizeof(image)) ||
        (image.part.part_id != part_id)) {
        return 0;
    }
    if (image.crc != crc_16((const uint8_t *)&image.part, sizeof(image.part))) {
        return 0;
    }

    return (force_lut_apply_part_data(&image.part) == FORCE_LUT_OK) ? 1 : 0;
}

uint8_t force_lut_flash_load_all(void)
{
    uint8_t index;
    uint8_t loaded = 0U;

    force_lut_clear_all_parts();

    for (index = 0U; index < FORCE_LUT_PART_COUNT; ++index) {
        if (force_lut_flash_load_part(force_lut_part_id_by_index(index)) != 0) {
            ++loaded;
        }
    }

    return loaded;
}
