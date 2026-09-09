#ifndef APPLICATION_FORCE_LUT_STORAGE_H
#define APPLICATION_FORCE_LUT_STORAGE_H

#include "force_lut_calculator.h"

#include <stdint.h>

#define FORCE_LUT_FLASH_SLOT_SIZE 2048U
#define FORCE_LUT_FLASH_MAGIC     0x544C5546UL
#define FORCE_LUT_FLASH_VERSION   1U

typedef struct {
    uint8_t  part_id;
    uint8_t  has_shape_feature;
    uint16_t sensor_rows;
    uint16_t sensor_cols;
    uint16_t area_num;
    uint16_t force_num;
    float area_cx[FORCE_LUT_MAX_AREA_NUM];
    float area_cy[FORCE_LUT_MAX_AREA_NUM];
    float force_grid[FORCE_LUT_MAX_FORCE_NUM];
    float sensor_table[FORCE_LUT_MAX_TABLE_COUNT];
} force_lut_part_data_t;

int force_lut_part_index(uint8_t part_id);
int force_lut_part_data_valid(const force_lut_part_data_t *part);
int force_lut_apply_part_data(const force_lut_part_data_t *part);
int force_lut_flash_write_part(const force_lut_part_data_t *part);
int force_lut_flash_load_part(uint8_t part_id);
uint8_t force_lut_flash_load_all(void);

#endif
