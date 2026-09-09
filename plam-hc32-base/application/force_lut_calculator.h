#ifndef FORCE_LUT_CALCULATOR_H
#define FORCE_LUT_CALCULATOR_H

/*
 * force_lut_calculator.h
 *
 * MCU 端通用尺寸力值预测模块。
 *
 * 当前算法：
 *   1) raw[rows*cols] 动态特征提取，兼容 6x4、8x10、12x8 等尺寸；
 *   2) 每个标定 area 保存一条 sensor_sum -> force 曲线；
 *   3) 同时可保存 area_eff / active_count / peak 等接触形态曲线；
 *   4) 运行时对所有 area 先分别反查 force，再按“位置距离 + 接触面积差异”做 K 近邻加权；
 *   5) 如果旧 CSV/旧参数没有接触形态曲线，则自动退化为纯位置 KNN 加权 LUT。
 *
 * 设计约束：C99、无 malloc/free、无 printf、固定容量静态数组。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FORCE_LUT_MAX_AREA_NUM
#define FORCE_LUT_MAX_AREA_NUM 20u
#endif

#ifndef FORCE_LUT_MAX_FORCE_NUM
#define FORCE_LUT_MAX_FORCE_NUM 12u
#endif

#ifndef FORCE_LUT_PART_COUNT
#define FORCE_LUT_PART_COUNT 6u
#endif

/* K 近邻融合数量。建议 3~4；标定点较少时会自动取 min(K, area_num)。 */
#ifndef FORCE_LUT_K_NEIGHBOR
#define FORCE_LUT_K_NEIGHBOR 4u
#endif

/* raw 点值大于该阈值才计入 active_count；sensor_sum/cx/cy 仍使用所有正值。 */
#ifndef FORCE_LUT_ACTIVE_VALUE_THRESHOLD
#define FORCE_LUT_ACTIVE_VALUE_THRESHOLD 0.0f
#endif

/* KNN 权重参数。位置为 0~1 归一化坐标，area_eff/peak 使用相对差异。 */
#ifndef FORCE_LUT_POS_WEIGHT
#define FORCE_LUT_POS_WEIGHT 1.0f
#endif
#ifndef FORCE_LUT_AREA_EFF_WEIGHT
#define FORCE_LUT_AREA_EFF_WEIGHT 0.25f
#endif
#ifndef FORCE_LUT_ACTIVE_COUNT_WEIGHT
#define FORCE_LUT_ACTIVE_COUNT_WEIGHT 0.15f
#endif
#ifndef FORCE_LUT_PEAK_RATIO_WEIGHT
#define FORCE_LUT_PEAK_RATIO_WEIGHT 0.10f
#endif
#ifndef FORCE_LUT_WEIGHT_EPSILON
#define FORCE_LUT_WEIGHT_EPSILON 1.0e-5f
#endif

/* 为兼容旧工程中可能仍引用的宏，保留但不再用于二维网格建表。 */
#ifndef FORCE_LUT_MAX_CX_NUM
#define FORCE_LUT_MAX_CX_NUM FORCE_LUT_MAX_AREA_NUM
#endif
#ifndef FORCE_LUT_MAX_CY_NUM
#define FORCE_LUT_MAX_CY_NUM 1u
#endif
#define FORCE_LUT_MAX_TABLE_COUNT (FORCE_LUT_MAX_AREA_NUM * FORCE_LUT_MAX_FORCE_NUM)

#ifndef FORCE_LUT_AREA_WARN_DISTANCE
#define FORCE_LUT_AREA_WARN_DISTANCE (0.08f)
#endif

#ifndef FORCE_LUT_AREA_REJECT_DISTANCE
#define FORCE_LUT_AREA_REJECT_DISTANCE (0.12f)
#endif

typedef enum ForceLutStatusC
{
    FORCE_LUT_OK                = 0,
    FORCE_LUT_ERR_NULL          = -1,
    FORCE_LUT_ERR_DIMENSION     = -2,
    FORCE_LUT_ERR_INVALID_AXIS  = -3,
    FORCE_LUT_ERR_NOT_LOADED    = -4,
    FORCE_LUT_ERR_OUT_OF_AREA   = -5,
    FORCE_LUT_ERR_NOT_TRIGGERED = -6
} ForceLutStatusC;

#ifndef FORCE_LUT_INVALID_RESULT
#define FORCE_LUT_INVALID_RESULT (-999999.0f)
#endif

typedef struct ForceLutRuntimeFeatureC
{
    uint8_t  triggered;
    uint16_t rows;
    uint16_t cols;

    float sensor_sum;
    float cx;
    float cy;
    float touch_weight;

    float peak_value;
    float active_count;
    float area_eff;
    float var_x;
    float var_y;
} ForceLutRuntimeFeatureC;

/* raw 按行优先排列，长度为 rows * cols。 */
int force_lut_extract_feature_u16(const uint16_t *raw, uint16_t rows, uint16_t cols,
                                  ForceLutRuntimeFeatureC *out);

/* 旧加载接口：只加载 area center + sensor_sum(force)，运行时使用纯位置 KNN 加权。 */
int force_lut_load_area_table_for_part(uint8_t part_id, uint16_t area_num, uint16_t force_num,
                                       const float *area_cx_in, const float *area_cy_in,
                                       const float *force_grid_in, const float *sensor_table_in);

int force_lut_load_area_table_for_part_ex(uint8_t part_id, uint16_t sensor_rows, uint16_t sensor_cols,
                                          uint16_t area_num, uint16_t force_num, const float *area_cx_in,
                                          const float *area_cy_in, const float *force_grid_in,
                                          const float *sensor_table_in);

/* 新加载接口：加载接触形态曲线，用于面积/峰值补偿。可传 NULL 表示该特征不可用。 */
int force_lut_load_feature_table_for_part_ex(uint8_t part_id, uint16_t sensor_rows, uint16_t sensor_cols,
                                             uint16_t area_num, uint16_t force_num, const float *area_cx_in,
                                             const float *area_cy_in, const float *force_grid_in,
                                             const float *sensor_table_in, const float *area_eff_table_in,
                                             const float *active_count_table_in,
                                             const float *peak_table_in);

float force_lut_predict_pressure_for_part(uint8_t part_id, float diff_ad_now, float cx_now, float cy_now);

float force_lut_predict_pressure_for_part_from_feature(uint8_t                        part_id,
                                                       const ForceLutRuntimeFeatureC *feature);

float force_lut_predict_pressure_for_part_from_u16(uint8_t part_id, const uint16_t *raw, uint16_t rows,
                                                   uint16_t cols);

int  force_lut_is_part_loaded(uint8_t part_id);
void force_lut_clear_all_parts(void);

typedef struct ForceLutPredictDebugC
{
    uint8_t  success;
    uint8_t  part_id;
    uint16_t area_num;
    uint16_t force_num;

    uint16_t model_rows;
    uint16_t model_cols;
    uint16_t runtime_rows;
    uint16_t runtime_cols;

    float input_diff_ad;
    float input_cx;
    float input_cy;
    float input_peak;
    float input_active_count;
    float input_area_eff;
    float input_var_x;
    float input_var_y;

    uint16_t selected_area;
    float    selected_area_cx;
    float    selected_area_cy;
    float    selected_area_distance;
    uint8_t  area_warn;
    uint8_t  area_reject;

    uint8_t  feature_curve_available;
    uint8_t  used_neighbor_count;
    uint16_t neighbor_area[FORCE_LUT_K_NEIGHBOR];
    float    neighbor_weight[FORCE_LUT_K_NEIGHBOR];
    float    neighbor_force[FORCE_LUT_K_NEIGHBOR];
    float    neighbor_distance[FORCE_LUT_K_NEIGHBOR];
    float    neighbor_expected_area_eff[FORCE_LUT_K_NEIGHBOR];

    float force_grid[FORCE_LUT_MAX_FORCE_NUM];
    float sensor_curve[FORCE_LUT_MAX_FORCE_NUM];
    float area_eff_curve[FORCE_LUT_MAX_FORCE_NUM];

    uint16_t force_low;
    uint16_t force_high;
    float    force_t;
    float    force_low_grid;
    float    force_high_grid;
    float    sensor_low;
    float    sensor_high;

    float predicted_force;
} ForceLutPredictDebugC;

int force_lut_debug_predict_for_part(uint8_t part_id, float diff_ad_now, float cx_now, float cy_now,
                                     ForceLutPredictDebugC *out);

int force_lut_debug_predict_for_part_from_feature(uint8_t part_id, const ForceLutRuntimeFeatureC *feature,
                                                  ForceLutPredictDebugC *out);

int force_lut_debug_predict_for_part_from_u16(uint8_t part_id, const uint16_t *raw, uint16_t rows,
                                              uint16_t cols, ForceLutPredictDebugC *out);

/* 兼容旧接口：旧版二维网格接口仍保留，内部会把 cx_grid 当作 area_cx、
 * cy_grid[0] 当作 area_cy，把 diff_ad_table 解释为 [area][force]。 */
int force_lut_load_table_from_arrays(uint16_t cx_num, uint16_t cy_num, uint16_t force_num,
                                     const float *cx_grid_in, const float *cy_grid_in,
                                     const float *force_grid_in, const float *diff_ad_table_in);

float force_lut_predict_pressure(float diff_ad_now, float cx_now, float cy_now);

int force_lut_load_table_for_part(uint8_t part_id, uint16_t cx_num, uint16_t cy_num, uint16_t force_num,
                                  const float *cx_grid_in, const float *cy_grid_in,
                                  const float *force_grid_in, const float *diff_ad_table_in);

#ifdef __cplusplus
}
#endif

#endif /* FORCE_LUT_CALCULATOR_H */
