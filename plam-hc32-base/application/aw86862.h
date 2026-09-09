#ifndef APPLICATION_AW86862_H
#define APPLICATION_AW86862_H

#include <stdint.h>

#include "hc32_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t chip_id;
    uint8_t reg_status;
    int16_t adc_ch0;
    int16_t adc_ch1;
    uint16_t adc_raw_ch0;
    uint16_t adc_raw_ch1;
} aw86862_data_t;

typedef struct
{
    uint8_t adch0cr1_0;
    uint8_t adch0cr1_1;
    uint8_t adch0cr1_2;
    uint8_t adch0_bw;
} aw86862_gain_config_t;

typedef enum
{
    AW86862_PGA1_GAIN_1 = 1,
    AW86862_PGA1_GAIN_16 = 16,
    AW86862_PGA1_GAIN_32 = 32,
    AW86862_PGA1_GAIN_64 = 64,
    AW86862_PGA1_GAIN_128 = 128,
    AW86862_PGA1_GAIN_256 = 256,
} aw86862_pga1_gain_t;

#define AW86862_GAIN_DEFAULT_ADCH0CR1_0    (0x40U)
#define AW86862_GAIN_DEFAULT_ADCH0CR1_1    (0x0AU)
#define AW86862_GAIN_DEFAULT_ADCH0CR1_2    (0x64U)
#define AW86862_GAIN_DEFAULT_BW            (0x42U)

int32_t aw86862_port_init(void);
int32_t aw86862_init(void);
int32_t aw86862_read_data(aw86862_data_t *data);
void aw86862_get_default_gain_config(aw86862_gain_config_t *config);
int32_t aw86862_set_gain_config(const aw86862_gain_config_t *config);
const aw86862_gain_config_t *aw86862_get_gain_config(void);
int32_t aw86862_set_pga(uint16_t pga1_gain, uint8_t pga2_gain);

#ifdef __cplusplus
}
#endif

#endif
