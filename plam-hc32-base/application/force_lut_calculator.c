#include "force_lut_calculator.h"

#include <string.h>

#define FORCE_LUT_EPSILON   (1.0e-6f)
#define FORCE_LUT_BIG_FLOAT (3.4e38f)

typedef struct ForceAreaLutC
{
    float cx;
    float cy;
    float sensor[FORCE_LUT_MAX_FORCE_NUM];
    float area_eff[FORCE_LUT_MAX_FORCE_NUM];
    float active_count[FORCE_LUT_MAX_FORCE_NUM];
    float peak[FORCE_LUT_MAX_FORCE_NUM];
} ForceAreaLutC;

typedef struct ForcePartLutC
{
    uint8_t       loaded;
    uint8_t       has_area_eff;
    uint8_t       has_active_count;
    uint8_t       has_peak;
    uint16_t      sensor_rows;
    uint16_t      sensor_cols;
    uint16_t      area_num;
    uint16_t      force_num;
    float         force_grid[FORCE_LUT_MAX_FORCE_NUM];
    ForceAreaLutC areas[FORCE_LUT_MAX_AREA_NUM];
} ForcePartLutC;

static ForcePartLutC g_parts[FORCE_LUT_PART_COUNT];
static ForcePartLutC g_single;

static int part_slot(uint8_t part_id)
{
    switch (part_id) {
    case 0x11u: return 0;
    case 0x21u: return 1;
    case 0x31u: return 2;
    case 0x41u: return 3;
    case 0x51u: return 4;
    case 0x61u: return 5;
    default:    return -1;
    }
}

static float lut_absf(float x) { return x < 0.0f ? -x : x; }

static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

static float safe_rel_diff(float a, float b)
{
    const float denom = lut_absf(a) + lut_absf(b) + 1.0f;
    return (a - b) / denom;
}

static int force_axis_is_strict_increasing(const float *axis, uint16_t n)
{
    uint16_t i;
    if (axis == 0 || n == 0u) return 0;
    for (i = 1u; i < n; ++i) {
        if (axis[i] <= axis[i - 1u]) return 0;
    }
    return 1;
}

static float sqrt_approx(float x)
{
    float r;
    int   k;
    if (x <= 0.0f) return 0.0f;
    r = x > 1.0f ? x : 1.0f;
    for (k = 0; k < 8; ++k) r = 0.5f * (r + x / r);
    return r;
}

int force_lut_extract_feature_u16(const uint16_t *raw, uint16_t rows, uint16_t cols,
                                  ForceLutRuntimeFeatureC *out)
{
    uint16_t r;
    uint16_t c;
    float    sum_w        = 0.0f;
    float    sum_sq       = 0.0f;
    float    sum_x        = 0.0f;
    float    sum_y        = 0.0f;
    float    peak         = 0.0f;
    float    active_count = 0.0f;

    if (out == 0) return FORCE_LUT_ERR_NULL;
    memset(out, 0, sizeof(*out));
    out->rows = rows;
    out->cols = cols;

    if (raw == 0) return FORCE_LUT_ERR_NULL;
    if (rows == 0u || cols == 0u) return FORCE_LUT_ERR_DIMENSION;

    for (r = 0u; r < rows; ++r) {
        for (c = 0u; c < cols; ++c) {
            const uint32_t index = (uint32_t)r * (uint32_t)cols + (uint32_t)c;
            const float    w     = (float)raw[index];
            float          x_norm;
            float          y_norm;
            if (w <= 0.0f) continue;
            x_norm = cols > 1u ? ((float)c / (float)(cols - 1u)) : 0.0f;
            y_norm = rows > 1u ? ((float)r / (float)(rows - 1u)) : 0.0f;
            sum_w += w;
            sum_sq += w * w;
            sum_x += w * x_norm;
            sum_y += w * y_norm;
            if (w > peak) peak = w;
            if (w > FORCE_LUT_ACTIVE_VALUE_THRESHOLD) active_count += 1.0f;
        }
    }

    out->sensor_sum   = sum_w;
    out->touch_weight = sum_w;
    out->peak_value   = peak;
    out->active_count = active_count;

    if (sum_w <= FORCE_LUT_EPSILON) {
        out->triggered = 0u;
        return FORCE_LUT_ERR_NOT_TRIGGERED;
    }

    out->cx       = clamp01(sum_x / sum_w);
    out->cy       = clamp01(sum_y / sum_w);
    out->area_eff = sum_sq > FORCE_LUT_EPSILON ? (sum_w * sum_w) / sum_sq : 0.0f;

    {
        float vx = 0.0f;
        float vy = 0.0f;
        for (r = 0u; r < rows; ++r) {
            for (c = 0u; c < cols; ++c) {
                const uint32_t index = (uint32_t)r * (uint32_t)cols + (uint32_t)c;
                const float    w     = (float)raw[index];
                float          x_norm;
                float          y_norm;
                float          dx;
                float          dy;
                if (w <= 0.0f) continue;
                x_norm = cols > 1u ? ((float)c / (float)(cols - 1u)) : 0.0f;
                y_norm = rows > 1u ? ((float)r / (float)(rows - 1u)) : 0.0f;
                dx     = x_norm - out->cx;
                dy     = y_norm - out->cy;
                vx += dx * dx * w;
                vy += dy * dy * w;
            }
        }
        out->var_x = vx / sum_w;
        out->var_y = vy / sum_w;
    }

    out->triggered = 1u;
    return FORCE_LUT_OK;
}

static float interpolate_curve_by_force(const ForcePartLutC *rt, const float *curve, float force)
{
    uint16_t       k;
    const uint16_t n = rt->force_num;
    if (n == 0u) return 0.0f;
    if (n == 1u) return curve[0];
    if (force <= rt->force_grid[0]) return curve[0];
    if (force >= rt->force_grid[n - 1u]) return curve[n - 1u];
    for (k = 0u; k < (uint16_t)(n - 1u); ++k) {
        const float f0 = rt->force_grid[k];
        const float f1 = rt->force_grid[k + 1u];
        if (force >= f0 && force <= f1) {
            const float t = (force - f0) / (f1 - f0);
            return curve[k] + t * (curve[k + 1u] - curve[k]);
        }
    }
    return curve[n - 1u];
}

static float reverse_lookup_force(const ForcePartLutC *rt, const float *sensor_curve, float sensor_now,
                                  uint16_t *force_low, uint16_t *force_high, float *force_t,
                                  float *sensor_low, float *sensor_high)
{
    uint16_t       k;
    const uint16_t n = rt->force_num;
    if (force_low) *force_low = 0u;
    if (force_high) *force_high = 0u;
    if (force_t) *force_t = 0.0f;
    if (sensor_low) *sensor_low = 0.0f;
    if (sensor_high) *sensor_high = 0.0f;

    if (n == 0u) return FORCE_LUT_INVALID_RESULT;
    if (n == 1u) {
        if (sensor_low) *sensor_low = sensor_curve[0];
        if (sensor_high) *sensor_high = sensor_curve[0];
        return rt->force_grid[0];
    }

    for (k = 0u; k < (uint16_t)(n - 1u); ++k) {
        const float s0    = sensor_curve[k];
        const float s1    = sensor_curve[k + 1u];
        const float f0    = rt->force_grid[k];
        const float f1    = rt->force_grid[k + 1u];
        const float min_s = s0 < s1 ? s0 : s1;
        const float max_s = s0 > s1 ? s0 : s1;
        if (sensor_now >= min_s && sensor_now <= max_s) {
            float t = 0.0f;
            if (lut_absf(s1 - s0) > FORCE_LUT_EPSILON) t = (sensor_now - s0) / (s1 - s0);
            if (force_low) *force_low = k;
            if (force_high) *force_high = (uint16_t)(k + 1u);
            if (force_t) *force_t = t;
            if (sensor_low) *sensor_low = s0;
            if (sensor_high) *sensor_high = s1;
            return f0 + t * (f1 - f0);
        }
    }

    {
        const float first_s = sensor_curve[0];
        const float last_s  = sensor_curve[n - 1u];
        if ((last_s >= first_s && sensor_now < first_s) || (last_s < first_s && sensor_now > first_s)) {
            if (force_low) *force_low = 0u;
            if (force_high) *force_high = 0u;
            if (sensor_low) *sensor_low = first_s;
            if (sensor_high) *sensor_high = first_s;
            return rt->force_grid[0];
        }
    }
    if (force_low) *force_low = (uint16_t)(n - 1u);
    if (force_high) *force_high = (uint16_t)(n - 1u);
    if (sensor_low) *sensor_low = sensor_curve[n - 1u];
    if (sensor_high) *sensor_high = sensor_curve[n - 1u];
    return rt->force_grid[n - 1u];
}

static int load_feature_table_runtime(ForcePartLutC *rt, uint16_t sensor_rows, uint16_t sensor_cols,
                                      uint16_t area_num, uint16_t force_num, const float *area_cx_in,
                                      const float *area_cy_in, const float *force_grid_in,
                                      const float *sensor_table_in, const float *area_eff_table_in,
                                      const float *active_count_table_in, const float *peak_table_in)
{
    uint16_t a, k;
    if (rt == 0 || area_cx_in == 0 || area_cy_in == 0 || force_grid_in == 0 || sensor_table_in == 0) {
        return FORCE_LUT_ERR_NULL;
    }
    if (area_num == 0u || area_num > FORCE_LUT_MAX_AREA_NUM || force_num == 0u ||
        force_num > FORCE_LUT_MAX_FORCE_NUM) {
        return FORCE_LUT_ERR_DIMENSION;
    }
    if (!force_axis_is_strict_increasing(force_grid_in, force_num)) {
        return FORCE_LUT_ERR_INVALID_AXIS;
    }

    memset(rt, 0, sizeof(*rt));
    rt->sensor_rows      = sensor_rows;
    rt->sensor_cols      = sensor_cols;
    rt->area_num         = area_num;
    rt->force_num        = force_num;
    rt->has_area_eff     = area_eff_table_in != 0 ? 1u : 0u;
    rt->has_active_count = active_count_table_in != 0 ? 1u : 0u;
    rt->has_peak         = peak_table_in != 0 ? 1u : 0u;

    for (k = 0u; k < force_num; ++k) rt->force_grid[k] = force_grid_in[k];
    for (a = 0u; a < area_num; ++a) {
        rt->areas[a].cx = clamp01(area_cx_in[a]);
        rt->areas[a].cy = clamp01(area_cy_in[a]);
        for (k = 0u; k < force_num; ++k) {
            const uint32_t idx           = (uint32_t)a * (uint32_t)force_num + (uint32_t)k;
            rt->areas[a].sensor[k]       = sensor_table_in[idx];
            rt->areas[a].area_eff[k]     = area_eff_table_in ? area_eff_table_in[idx] : 0.0f;
            rt->areas[a].active_count[k] = active_count_table_in ? active_count_table_in[idx] : 0.0f;
            rt->areas[a].peak[k]         = peak_table_in ? peak_table_in[idx] : 0.0f;
        }
    }
    rt->loaded = 1u;
    return FORCE_LUT_OK;
}

int force_lut_load_feature_table_for_part_ex(uint8_t part_id, uint16_t sensor_rows, uint16_t sensor_cols,
                                             uint16_t area_num, uint16_t force_num, const float *area_cx_in,
                                             const float *area_cy_in, const float *force_grid_in,
                                             const float *sensor_table_in, const float *area_eff_table_in,
                                             const float *active_count_table_in, const float *peak_table_in)
{
    const int slot = part_slot(part_id);
    if (slot < 0) return FORCE_LUT_ERR_DIMENSION;
    return load_feature_table_runtime(&g_parts[slot], sensor_rows, sensor_cols, area_num, force_num,
                                      area_cx_in, area_cy_in, force_grid_in, sensor_table_in,
                                      area_eff_table_in, active_count_table_in, peak_table_in);
}

int force_lut_load_area_table_for_part_ex(uint8_t part_id, uint16_t sensor_rows, uint16_t sensor_cols,
                                          uint16_t area_num, uint16_t force_num, const float *area_cx_in,
                                          const float *area_cy_in, const float *force_grid_in,
                                          const float *sensor_table_in)
{
    return force_lut_load_feature_table_for_part_ex(part_id, sensor_rows, sensor_cols, area_num, force_num,
                                                    area_cx_in, area_cy_in, force_grid_in, sensor_table_in,
                                                    0, 0, 0);
}

int force_lut_load_area_table_for_part(uint8_t part_id, uint16_t area_num, uint16_t force_num,
                                       const float *area_cx_in, const float *area_cy_in,
                                       const float *force_grid_in, const float *sensor_table_in)
{
    return force_lut_load_area_table_for_part_ex(part_id, 0u, 0u, area_num, force_num, area_cx_in,
                                                 area_cy_in, force_grid_in, sensor_table_in);
}

static void insert_neighbor(uint16_t area, float score, float weight, float force, float expected_area_eff,
                            uint8_t kmax, uint8_t *count, uint16_t *areas, float *scores, float *weights,
                            float *forces, float *expected_area_effs)
{
    uint8_t pos;
    if (kmax == 0u) return;
    if (*count < kmax) {
        pos = *count;
        ++(*count);
    }
    else {
        if (score >= scores[kmax - 1u]) return;
        pos = (uint8_t)(kmax - 1u);
    }
    while (pos > 0u && score < scores[pos - 1u]) {
        areas[pos]              = areas[pos - 1u];
        scores[pos]             = scores[pos - 1u];
        weights[pos]            = weights[pos - 1u];
        forces[pos]             = forces[pos - 1u];
        expected_area_effs[pos] = expected_area_effs[pos - 1u];
        --pos;
    }
    areas[pos]              = area;
    scores[pos]             = score;
    weights[pos]            = weight;
    forces[pos]             = force;
    expected_area_effs[pos] = expected_area_eff;
}

static int debug_predict_runtime_feature(const ForcePartLutC *rt, uint8_t part_id,
                                         const ForceLutRuntimeFeatureC *feature, ForceLutPredictDebugC *out)
{
    uint16_t      a;
    uint16_t      k;
    const uint8_t kmax =
        (FORCE_LUT_K_NEIGHBOR > FORCE_LUT_MAX_AREA_NUM) ? FORCE_LUT_MAX_AREA_NUM : FORCE_LUT_K_NEIGHBOR;
    uint8_t  neighbor_count = 0u;
    uint16_t n_area[FORCE_LUT_K_NEIGHBOR];
    float    n_score[FORCE_LUT_K_NEIGHBOR];
    float    n_weight[FORCE_LUT_K_NEIGHBOR];
    float    n_force[FORCE_LUT_K_NEIGHBOR];
    float    n_expected_area_eff[FORCE_LUT_K_NEIGHBOR];
    float    weight_sum = 0.0f;
    float    force_sum  = 0.0f;

    if (out == 0 || feature == 0) return FORCE_LUT_ERR_NULL;
    memset(out, 0, sizeof(*out));
    out->part_id            = part_id;
    out->runtime_rows       = feature->rows;
    out->runtime_cols       = feature->cols;
    out->input_diff_ad      = feature->sensor_sum;
    out->input_cx           = feature->cx;
    out->input_cy           = feature->cy;
    out->input_peak         = feature->peak_value;
    out->input_active_count = feature->active_count;
    out->input_area_eff     = feature->area_eff;
    out->input_var_x        = feature->var_x;
    out->input_var_y        = feature->var_y;

    if (rt == 0 || !rt->loaded) return FORCE_LUT_ERR_NOT_LOADED;

    out->model_rows              = rt->sensor_rows;
    out->model_cols              = rt->sensor_cols;
    out->area_num                = rt->area_num;
    out->force_num               = rt->force_num;
    out->feature_curve_available = (rt->has_area_eff || rt->has_peak || rt->has_active_count) ? 1u : 0u;

    for (k = 0u; k < kmax; ++k) {
        n_area[k]              = 0u;
        n_score[k]             = FORCE_LUT_BIG_FLOAT;
        n_weight[k]            = 0.0f;
        n_force[k]             = 0.0f;
        n_expected_area_eff[k] = 0.0f;
    }

    for (a = 0u; a < rt->area_num; ++a) {
        uint16_t    low = 0u, high = 0u;
        float       t = 0.0f, s_low = 0.0f, s_high = 0.0f;
        const float f  = reverse_lookup_force(rt, rt->areas[a].sensor, feature->sensor_sum, &low, &high, &t,
                                              &s_low, &s_high);
        const float dx = feature->cx - rt->areas[a].cx;
        const float dy = feature->cy - rt->areas[a].cy;
        float       score             = FORCE_LUT_POS_WEIGHT * (dx * dx + dy * dy);
        float       expected_area_eff = 0.0f;
        float       weight;

        (void)low;
        (void)high;
        (void)t;
        (void)s_low;
        (void)s_high;
        if (f == FORCE_LUT_INVALID_RESULT) continue;

        if (rt->has_area_eff) {
            const float expected = interpolate_curve_by_force(rt, rt->areas[a].area_eff, f);
            const float d        = safe_rel_diff(feature->area_eff, expected);
            expected_area_eff    = expected;
            score += FORCE_LUT_AREA_EFF_WEIGHT * d * d;
        }
        if (rt->has_active_count) {
            const float expected_active = interpolate_curve_by_force(rt, rt->areas[a].active_count, f);
            const float d               = safe_rel_diff(feature->active_count, expected_active);
            score += FORCE_LUT_ACTIVE_COUNT_WEIGHT * d * d;
        }
        if (rt->has_peak) {
            const float expected_peak = interpolate_curve_by_force(rt, rt->areas[a].peak, f);
            const float d             = safe_rel_diff(feature->peak_value, expected_peak);
            score += FORCE_LUT_PEAK_RATIO_WEIGHT * d * d;
        }

        weight = 1.0f / (score + FORCE_LUT_WEIGHT_EPSILON);
        insert_neighbor(a, score, weight, f, expected_area_eff, kmax, &neighbor_count, n_area, n_score,
                        n_weight, n_force, n_expected_area_eff);
    }

    if (neighbor_count == 0u) return FORCE_LUT_ERR_INVALID_AXIS;

    for (k = 0u; k < neighbor_count; ++k) {
        weight_sum += n_weight[k];
        force_sum += n_weight[k] * n_force[k];
    }
    if (weight_sum <= FORCE_LUT_EPSILON) return FORCE_LUT_ERR_INVALID_AXIS;

    out->used_neighbor_count    = neighbor_count;
    out->selected_area          = n_area[0];
    out->selected_area_cx       = rt->areas[out->selected_area].cx;
    out->selected_area_cy       = rt->areas[out->selected_area].cy;
    out->selected_area_distance = sqrt_approx(n_score[0]);
    out->area_warn              = out->selected_area_distance > FORCE_LUT_AREA_WARN_DISTANCE ? 1u : 0u;
    out->area_reject            = out->selected_area_distance > FORCE_LUT_AREA_REJECT_DISTANCE ? 1u : 0u;

    for (k = 0u; k < neighbor_count; ++k) {
        out->neighbor_area[k]              = n_area[k];
        out->neighbor_weight[k]            = n_weight[k] / weight_sum;
        out->neighbor_force[k]             = n_force[k];
        out->neighbor_distance[k]          = sqrt_approx(n_score[k]);
        out->neighbor_expected_area_eff[k] = n_expected_area_eff[k];
    }

    for (k = 0u; k < rt->force_num; ++k) {
        out->force_grid[k]     = rt->force_grid[k];
        out->sensor_curve[k]   = rt->areas[out->selected_area].sensor[k];
        out->area_eff_curve[k] = rt->areas[out->selected_area].area_eff[k];
    }

    out->predicted_force =
        reverse_lookup_force(rt, rt->areas[out->selected_area].sensor, feature->sensor_sum, &out->force_low,
                             &out->force_high, &out->force_t, &out->sensor_low, &out->sensor_high);
    out->force_low_grid  = rt->force_grid[out->force_low];
    out->force_high_grid = rt->force_grid[out->force_high];

    /* 最终输出使用 KNN 融合值，selected_area 反查值仅用于日志展示。 */
    out->predicted_force = force_sum / weight_sum;
    out->success         = 1u;
    return FORCE_LUT_OK;
}

float force_lut_predict_pressure_for_part(uint8_t part_id, float diff_ad_now, float cx_now, float cy_now)
{
    ForceLutRuntimeFeatureC feature;
    ForceLutPredictDebugC   dbg;
    memset(&feature, 0, sizeof(feature));
    feature.triggered    = diff_ad_now > FORCE_LUT_EPSILON ? 1u : 0u;
    feature.sensor_sum   = diff_ad_now;
    feature.touch_weight = diff_ad_now;
    feature.cx           = clamp01(cx_now);
    feature.cy           = clamp01(cy_now);
    if (!feature.triggered) return FORCE_LUT_INVALID_RESULT;
    if (force_lut_debug_predict_for_part_from_feature(part_id, &feature, &dbg) != FORCE_LUT_OK ||
        !dbg.success) {
        return FORCE_LUT_INVALID_RESULT;
    }
    return dbg.predicted_force;
}

float force_lut_predict_pressure_for_part_from_feature(uint8_t                        part_id,
                                                       const ForceLutRuntimeFeatureC *feature)
{
    ForceLutPredictDebugC dbg;
    if (force_lut_debug_predict_for_part_from_feature(part_id, feature, &dbg) != FORCE_LUT_OK ||
        !dbg.success) {
        return FORCE_LUT_INVALID_RESULT;
    }
    return dbg.predicted_force;
}

float force_lut_predict_pressure_for_part_from_u16(uint8_t part_id, const uint16_t *raw, uint16_t rows,
                                                   uint16_t cols)
{
    ForceLutPredictDebugC dbg;
    if (force_lut_debug_predict_for_part_from_u16(part_id, raw, rows, cols, &dbg) != FORCE_LUT_OK ||
        !dbg.success) {
        return FORCE_LUT_INVALID_RESULT;
    }
    return dbg.predicted_force;
}

int force_lut_debug_predict_for_part(uint8_t part_id, float diff_ad_now, float cx_now, float cy_now,
                                     ForceLutPredictDebugC *out)
{
    ForceLutRuntimeFeatureC feature;
    memset(&feature, 0, sizeof(feature));
    feature.triggered    = diff_ad_now > FORCE_LUT_EPSILON ? 1u : 0u;
    feature.sensor_sum   = diff_ad_now;
    feature.touch_weight = diff_ad_now;
    feature.cx           = clamp01(cx_now);
    feature.cy           = clamp01(cy_now);
    return force_lut_debug_predict_for_part_from_feature(part_id, &feature, out);
}

int force_lut_debug_predict_for_part_from_feature(uint8_t part_id, const ForceLutRuntimeFeatureC *feature,
                                                  ForceLutPredictDebugC *out)
{
    const int slot = part_slot(part_id);
    if (feature == 0 || out == 0) return FORCE_LUT_ERR_NULL;
    if (!feature->triggered) return FORCE_LUT_ERR_NOT_TRIGGERED;
    if (slot < 0) return FORCE_LUT_ERR_DIMENSION;
    return debug_predict_runtime_feature(&g_parts[slot], part_id, feature, out);
}

int force_lut_debug_predict_for_part_from_u16(uint8_t part_id, const uint16_t *raw, uint16_t rows,
                                              uint16_t cols, ForceLutPredictDebugC *out)
{
    ForceLutRuntimeFeatureC feature;
    const int               status = force_lut_extract_feature_u16(raw, rows, cols, &feature);
    if (status != FORCE_LUT_OK) {
        if (out) {
            memset(out, 0, sizeof(*out));
            out->part_id      = part_id;
            out->runtime_rows = rows;
            out->runtime_cols = cols;
        }
        return status;
    }
    return force_lut_debug_predict_for_part_from_feature(part_id, &feature, out);
}

int force_lut_is_part_loaded(uint8_t part_id)
{
    const int slot = part_slot(part_id);
    return (slot >= 0 && g_parts[slot].loaded) ? 1 : 0;
}

void force_lut_clear_all_parts(void)
{
    memset(g_parts, 0, sizeof(g_parts));
    memset(&g_single, 0, sizeof(g_single));
}

int force_lut_load_table_from_arrays(uint16_t cx_num, uint16_t cy_num, uint16_t force_num,
                                     const float *cx_grid_in, const float *cy_grid_in,
                                     const float *force_grid_in, const float *diff_ad_table_in)
{
    uint16_t a, k;
    float    area_cy[FORCE_LUT_MAX_AREA_NUM];
    float    compact_table[FORCE_LUT_MAX_TABLE_COUNT];
    if (cx_grid_in == 0 || cy_grid_in == 0 || force_grid_in == 0 || diff_ad_table_in == 0) {
        return FORCE_LUT_ERR_NULL;
    }
    if (cx_num == 0u || cx_num > FORCE_LUT_MAX_AREA_NUM || force_num == 0u ||
        force_num > FORCE_LUT_MAX_FORCE_NUM) {
        return FORCE_LUT_ERR_DIMENSION;
    }
    (void)cy_num;
    for (a = 0u; a < cx_num; ++a) {
        area_cy[a] = cy_grid_in[0];
        for (k = 0u; k < force_num; ++k) {
            compact_table[(uint32_t)a * (uint32_t)force_num + (uint32_t)k] =
                diff_ad_table_in[(uint32_t)a * (uint32_t)force_num + (uint32_t)k];
        }
    }
    return load_feature_table_runtime(&g_single, 0u, 0u, cx_num, force_num, cx_grid_in, area_cy,
                                      force_grid_in, compact_table, 0, 0, 0);
}

float force_lut_predict_pressure(float diff_ad_now, float cx_now, float cy_now)
{
    ForceLutRuntimeFeatureC feature;
    ForceLutPredictDebugC   dbg;
    memset(&feature, 0, sizeof(feature));
    feature.triggered  = diff_ad_now > FORCE_LUT_EPSILON ? 1u : 0u;
    feature.sensor_sum = diff_ad_now;
    feature.cx         = clamp01(cx_now);
    feature.cy         = clamp01(cy_now);
    if (debug_predict_runtime_feature(&g_single, 0u, &feature, &dbg) != FORCE_LUT_OK || !dbg.success) {
        return FORCE_LUT_INVALID_RESULT;
    }
    return dbg.predicted_force;
}

int force_lut_load_table_for_part(uint8_t part_id, uint16_t cx_num, uint16_t cy_num, uint16_t force_num,
                                  const float *cx_grid_in, const float *cy_grid_in,
                                  const float *force_grid_in, const float *diff_ad_table_in)
{
    uint16_t a, k;
    float    area_cy[FORCE_LUT_MAX_AREA_NUM];
    float    compact_table[FORCE_LUT_MAX_TABLE_COUNT];
    if (cx_grid_in == 0 || cy_grid_in == 0 || force_grid_in == 0 || diff_ad_table_in == 0) {
        return FORCE_LUT_ERR_NULL;
    }
    if (cx_num == 0u || cx_num > FORCE_LUT_MAX_AREA_NUM || force_num == 0u ||
        force_num > FORCE_LUT_MAX_FORCE_NUM) {
        return FORCE_LUT_ERR_DIMENSION;
    }
    (void)cy_num;
    for (a = 0u; a < cx_num; ++a) {
        area_cy[a] = cy_grid_in[0];
        for (k = 0u; k < force_num; ++k) {
            compact_table[(uint32_t)a * (uint32_t)force_num + (uint32_t)k] =
                diff_ad_table_in[(uint32_t)a * (uint32_t)force_num + (uint32_t)k];
        }
    }
    return force_lut_load_area_table_for_part(part_id, cx_num, force_num, cx_grid_in, area_cy,
                                              force_grid_in, compact_table);
}
