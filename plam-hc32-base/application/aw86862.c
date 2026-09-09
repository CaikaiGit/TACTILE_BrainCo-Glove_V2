#include "aw86862.h"

#include "aw86862_reg.h"

#include <stdbool.h>

#include "hc32_ll_clk.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_i2c.h"
#include "hc32_ll_utility.h"

#define AW86862_I2C_UNIT             (CM_I2C2)
#define AW86862_I2C_FCG              (FCG1_PERIPH_I2C2)
#define AW86862_I2C_ADDR             (0x6AU)
#define AW86862_I2C_BAUDRATE         (400000UL)
#define AW86862_I2C_TIMEOUT          (0x40000UL)

#define AW86862_I2C_SCL_PORT         (GPIO_PORT_A)
#define AW86862_I2C_SCL_PIN          (GPIO_PIN_10)
#define AW86862_I2C_SCL_FUNC         (GPIO_FUNC_51)
#define AW86862_I2C_SDA_PORT         (GPIO_PORT_A)
#define AW86862_I2C_SDA_PIN          (GPIO_PIN_09)
#define AW86862_I2C_SDA_FUNC         (GPIO_FUNC_50)

#define AW86862_CHIPID_EXPECTED      (0x62U)
#define AW86862_WRITE_UNLOCK         (0xA5U)
#define AW86862_RESET_VALUE          (0x10U)
#define AW86862_INIT_DELAY_MS        (20U)
#define AW86862_ADC_DATA_BASE        (0x2000)
#define AW86862_ADMCR0_ENABLE_MASK   (1U << 5)
#define AW86862_ADMCR1_ADST          (0x08U)
#define AW86862_I2C_WAKE_MODE        (0x40U)
#define AW86862_CH0_CR0_0_DEFAULT    (0x01U)
#define AW86862_CH1_CR0_0_DEFAULT    (0x10U)
#define AW86862_CHCR0_1_DEFAULT      (0x90U)
#define AW86862_CHCR0_2_DEFAULT      (0x10U)
#define AW86862_CHCR0_3_DEFAULT      (0x10U)
#define AW86862_BASELINE_SAMPLES     (64U)
#define AW86862_BASELINE_DELAY_MS    (50U)

static const uint8_t m_au8Aw86862RegParam[] = {
    0x18U, 0x00U,
    0x19U, 0x00U,
    0x1AU, 0x00U,
    0x21U, 0x12U,
    0x30U, 0x01U,
    0x31U, 0xB0U,
    0xC2U, 0x2CU,
    0xCCU, 0x07U,
    0xCDU, 0x05U,
    0xCEU, 0x50U,
    0xCFU, 0x20U,
    0xD0U, 0x03U,
    0xD1U, 0x05U,
    0xD2U, 0xB0U,
    0xD3U, 0x1FU,
    0xD4U, 0x03U,
};

volatile uint32_t g_aw86862_i2c_src_clk;
volatile uint32_t g_aw86862_i2c_clk_div_calc;
volatile uint32_t g_aw86862_i2c_clk_div_reg;
static int16_t m_i16BaselineCh0;
static int16_t m_i16BaselineCh1;
static bool m_bBaselineValid;

/* Default values come from the official Awinic reference driver. */
static aw86862_gain_config_t m_stcGainConfig = {
    AW86862_GAIN_DEFAULT_ADCH0CR1_0,
    AW86862_GAIN_DEFAULT_ADCH0CR1_1,
    AW86862_GAIN_DEFAULT_ADCH0CR1_2,
    AW86862_GAIN_DEFAULT_BW,
};

static int32_t aw86862_i2c_init(void)
{
    int32_t i32Ret;
    float32_t fErr;
    stc_i2c_init_t stcI2cInit;
    uint32_t u32I2cSrcClk;
    uint32_t u32I2cClkDiv;
    uint32_t u32I2cClkDivReg;

    GPIO_SetFunc(AW86862_I2C_SCL_PORT, AW86862_I2C_SCL_PIN, AW86862_I2C_SCL_FUNC);
    GPIO_SetFunc(AW86862_I2C_SDA_PORT, AW86862_I2C_SDA_PIN, AW86862_I2C_SDA_FUNC);

    FCG_Fcg1PeriphClockCmd(AW86862_I2C_FCG, ENABLE);

    u32I2cSrcClk = I2C_SRC_CLK;
    u32I2cClkDiv = u32I2cSrcClk / AW86862_I2C_BAUDRATE / I2C_WIDTH_MAX_IMME;
    for (u32I2cClkDivReg = I2C_CLK_DIV1; u32I2cClkDivReg <= I2C_CLK_DIV128; ++u32I2cClkDivReg) {
        if (u32I2cClkDiv < (1UL << u32I2cClkDivReg)) {
            break;
        }
    }
    if (u32I2cClkDivReg > I2C_CLK_DIV128) {
        u32I2cClkDivReg = I2C_CLK_DIV128;
    }

    g_aw86862_i2c_src_clk = u32I2cSrcClk;
    g_aw86862_i2c_clk_div_calc = u32I2cClkDiv;
    g_aw86862_i2c_clk_div_reg = u32I2cClkDivReg;

    (void)I2C_DeInit(AW86862_I2C_UNIT);
    (void)I2C_StructInit(&stcI2cInit);
    stcI2cInit.u32Baudrate = AW86862_I2C_BAUDRATE;
    stcI2cInit.u32SclTime = (uint32_t)((uint64_t)250UL *
                                       ((uint64_t)u32I2cSrcClk / ((uint64_t)1UL << u32I2cClkDivReg)) /
                                       (uint64_t)1000000000UL);
    stcI2cInit.u32ClockDiv = u32I2cClkDivReg;
    i32Ret = I2C_Init(AW86862_I2C_UNIT, &stcI2cInit, &fErr);
    if (LL_OK == i32Ret) {
        I2C_BusWaitCmd(AW86862_I2C_UNIT, ENABLE);
        I2C_Cmd(AW86862_I2C_UNIT, ENABLE);
    }

    return i32Ret;
}

static int32_t aw86862_i2c_write(uint8_t reg, const uint8_t *buf, uint32_t len)
{
    int32_t i32Ret;

    I2C_SWResetCmd(AW86862_I2C_UNIT, ENABLE);
    I2C_SWResetCmd(AW86862_I2C_UNIT, DISABLE);
    i32Ret = I2C_Start(AW86862_I2C_UNIT, AW86862_I2C_TIMEOUT);
    if (LL_OK == i32Ret) {
        i32Ret = I2C_TransAddr(AW86862_I2C_UNIT, AW86862_I2C_ADDR, I2C_DIR_TX, AW86862_I2C_TIMEOUT);
        if (LL_OK == i32Ret) {
            i32Ret = I2C_TransData(AW86862_I2C_UNIT, &reg, 1U, AW86862_I2C_TIMEOUT);
            if ((LL_OK == i32Ret) && (len > 0U)) {
                i32Ret = I2C_TransData(AW86862_I2C_UNIT, buf, len, AW86862_I2C_TIMEOUT);
            }
        }
    }
    (void)I2C_Stop(AW86862_I2C_UNIT, AW86862_I2C_TIMEOUT);
    return i32Ret;
}

static int32_t aw86862_i2c_read(uint8_t reg, uint8_t *buf, uint32_t len)
{
    int32_t i32Ret;

    I2C_SWResetCmd(AW86862_I2C_UNIT, ENABLE);
    I2C_SWResetCmd(AW86862_I2C_UNIT, DISABLE);
    i32Ret = I2C_Start(AW86862_I2C_UNIT, AW86862_I2C_TIMEOUT);
    if (LL_OK == i32Ret) {
        i32Ret = I2C_TransAddr(AW86862_I2C_UNIT, AW86862_I2C_ADDR, I2C_DIR_TX, AW86862_I2C_TIMEOUT);
        if (LL_OK == i32Ret) {
            i32Ret = I2C_TransData(AW86862_I2C_UNIT, &reg, 1U, AW86862_I2C_TIMEOUT);
            if (LL_OK == i32Ret) {
                i32Ret = I2C_Restart(AW86862_I2C_UNIT, AW86862_I2C_TIMEOUT);
                if (LL_OK == i32Ret) {
                    if (1UL == len) {
                        I2C_AckConfig(AW86862_I2C_UNIT, I2C_NACK);
                    }
                    i32Ret = I2C_TransAddr(AW86862_I2C_UNIT, AW86862_I2C_ADDR, I2C_DIR_RX, AW86862_I2C_TIMEOUT);
                    if (LL_OK == i32Ret) {
                        i32Ret = I2C_MasterReceiveDataAndStop(AW86862_I2C_UNIT, buf, len, AW86862_I2C_TIMEOUT);
                    }
                    I2C_AckConfig(AW86862_I2C_UNIT, I2C_ACK);
                }
            }
        }
    }

    if (LL_OK != i32Ret) {
        (void)I2C_Stop(AW86862_I2C_UNIT, AW86862_I2C_TIMEOUT);
    }
    return i32Ret;
}

static int32_t aw86862_write_byte(uint8_t reg, uint8_t value)
{
    return aw86862_i2c_write(reg, &value, 1U);
}

static int32_t aw86862_pga1_gain_to_reg(uint16_t gain, uint8_t *value)
{
    if (value == NULL) {
        return LL_ERR_INVD_PARAM;
    }

    switch (gain) {
        case AW86862_PGA1_GAIN_1:
            *value = (0U << 4);
            break;
        case AW86862_PGA1_GAIN_16:
            *value = (1U << 4);
            break;
        case AW86862_PGA1_GAIN_32:
            *value = (2U << 4);
            break;
        case AW86862_PGA1_GAIN_64:
            *value = (3U << 4);
            break;
        case AW86862_PGA1_GAIN_128:
            *value = (4U << 4);
            break;
        case AW86862_PGA1_GAIN_256:
            *value = (5U << 4);
            break;
        default:
            return LL_ERR_INVD_PARAM;
    }

    return LL_OK;
}

static int32_t aw86862_read_byte(uint8_t reg, uint8_t *value)
{
    return aw86862_i2c_read(reg, value, 1U);
}

static int32_t aw86862_read_raw_data(aw86862_data_t *data)
{
    uint8_t raw[2];
    int32_t i32Ret;

    i32Ret = aw86862_read_byte(REG_CHIPID, &data->chip_id);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    i32Ret = aw86862_read_byte(REG_ADSR_0, &data->reg_status);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    i32Ret = aw86862_i2c_read(REG_ADCH0DR_0, raw, 2U);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    data->adc_raw_ch0 = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8U);
    data->adc_ch0 = (int16_t)data->adc_raw_ch0 - AW86862_ADC_DATA_BASE;

    i32Ret = aw86862_i2c_read(REG_ADCH1DR_0, raw, 2U);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    data->adc_raw_ch1 = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8U);
    data->adc_ch1 = (int16_t)data->adc_raw_ch1 - AW86862_ADC_DATA_BASE;
    return LL_OK;
}

static void aw86862_baseline_reset(void)
{
    m_i16BaselineCh0 = 0;
    m_i16BaselineCh1 = 0;
    m_bBaselineValid = false;
}

static int32_t aw86862_capture_baseline(uint16_t samples)
{
    aw86862_data_t stcData;
    int32_t i32Ret;
    int32_t i32SumCh0 = 0;
    int32_t i32SumCh1 = 0;
    uint16_t u16ValidSamples = 0;
    uint16_t i;

    for (i = 0U; i < samples; ++i) {
        i32Ret = aw86862_read_raw_data(&stcData);
        if (LL_OK == i32Ret) {
            i32SumCh0 += stcData.adc_ch0;
            i32SumCh1 += stcData.adc_ch1;
            ++u16ValidSamples;
        }
        DDL_DelayMS(1U);
    }

    if (0U == u16ValidSamples) {
        return LL_ERR;
    }

    m_i16BaselineCh0 = (int16_t)(i32SumCh0 / (int32_t)u16ValidSamples);
    m_i16BaselineCh1 = (int16_t)(i32SumCh1 / (int32_t)u16ValidSamples);
    m_bBaselineValid = true;
    return LL_OK;
}

static int32_t aw86862_write_bits(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t regValue;
    int32_t i32Ret;

    i32Ret = aw86862_read_byte(reg, &regValue);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    regValue &= mask;
    regValue |= value;
    return aw86862_write_byte(reg, regValue);
}

static int32_t aw86862_load_param_table(void)
{
    uint32_t i;

    if (LL_OK != aw86862_write_byte(REG_WR_UNLOCK, AW86862_WRITE_UNLOCK)) {
        return LL_ERR;
    }

    for (i = 0U; i < (sizeof(m_au8Aw86862RegParam) / 2U); ++i) {
        if (LL_OK != aw86862_write_byte(m_au8Aw86862RegParam[i * 2U], m_au8Aw86862RegParam[i * 2U + 1U])) {
            return LL_ERR;
        }
    }

    if (LL_OK != aw86862_write_byte(REG_ADCH0CR1_0, m_stcGainConfig.adch0cr1_0)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH0CR1_1, m_stcGainConfig.adch0cr1_1)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH0CR1_2, m_stcGainConfig.adch0cr1_2)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH0_BW, m_stcGainConfig.adch0_bw)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH0CR0_0, AW86862_CH0_CR0_0_DEFAULT)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH0CR0_1, AW86862_CHCR0_1_DEFAULT)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH0CR0_2, AW86862_CHCR0_2_DEFAULT)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH0CR0_3, AW86862_CHCR0_3_DEFAULT)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH1CR0_0, AW86862_CH1_CR0_0_DEFAULT)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH1CR0_1, AW86862_CHCR0_1_DEFAULT)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH1CR0_2, AW86862_CHCR0_2_DEFAULT)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH1CR0_3, AW86862_CHCR0_3_DEFAULT)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH1CR1_0, m_stcGainConfig.adch0cr1_0)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH1CR1_1, m_stcGainConfig.adch0cr1_1)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH1CR1_2, m_stcGainConfig.adch0cr1_2)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADCH1_BW, m_stcGainConfig.adch0_bw)) {
        return LL_ERR;
    }

    return LL_OK;
}

static int32_t aw86862_start_adc(void)
{
    if (LL_OK != aw86862_write_bits(REG_ADMCR_0, (uint8_t)(~AW86862_ADMCR0_ENABLE_MASK), AW86862_ADMCR0_ENABLE_MASK)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_ADMCR_1, AW86862_ADMCR1_ADST)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_GO_STDBY_CFG, 0U)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_LD_SWDT_CFG, AW86862_I2C_WAKE_MODE)) {
        return LL_ERR;
    }

    return LL_OK;
}

int32_t aw86862_port_init(void)
{
    return aw86862_i2c_init();
}

int32_t aw86862_init(void)
{
    uint8_t chipId;

    if (LL_OK != aw86862_read_byte(REG_CHIPID, &chipId)) {
        return LL_ERR;
    }
    if (chipId != AW86862_CHIPID_EXPECTED) {
        return LL_ERR;
    }

    if (LL_OK != aw86862_write_byte(REG_WR_UNLOCK, AW86862_WRITE_UNLOCK)) {
        return LL_ERR;
    }
    if (LL_OK != aw86862_write_byte(REG_RST_CFG, AW86862_RESET_VALUE)) {
        return LL_ERR;
    }
    DDL_DelayMS(AW86862_INIT_DELAY_MS);

    if (LL_OK != aw86862_load_param_table()) {
        return LL_ERR;
    }

    aw86862_baseline_reset();
    if (LL_OK != aw86862_start_adc()) {
        return LL_ERR;
    }

    DDL_DelayMS(AW86862_BASELINE_DELAY_MS);
    return aw86862_capture_baseline(AW86862_BASELINE_SAMPLES);
}

int32_t aw86862_read_data(aw86862_data_t *data)
{
    int32_t i32Ret;

    if (data == NULL) {
        return LL_ERR_INVD_PARAM;
    }

    i32Ret = aw86862_read_raw_data(data);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    if (m_bBaselineValid) {
        data->adc_ch0 = (int16_t)(data->adc_ch0 - m_i16BaselineCh0);
        data->adc_ch1 = (int16_t)(data->adc_ch1 - m_i16BaselineCh1);
    }

    return LL_OK;
}

void aw86862_get_default_gain_config(aw86862_gain_config_t *config)
{
    if (config != NULL) {
        config->adch0cr1_0 = AW86862_GAIN_DEFAULT_ADCH0CR1_0;
        config->adch0cr1_1 = AW86862_GAIN_DEFAULT_ADCH0CR1_1;
        config->adch0cr1_2 = AW86862_GAIN_DEFAULT_ADCH0CR1_2;
        config->adch0_bw = AW86862_GAIN_DEFAULT_BW;
    }
}

int32_t aw86862_set_gain_config(const aw86862_gain_config_t *config)
{
    if (config == NULL) {
        return LL_ERR_INVD_PARAM;
    }

    m_stcGainConfig = *config;
    return LL_OK;
}

const aw86862_gain_config_t *aw86862_get_gain_config(void)
{
    return &m_stcGainConfig;
}

int32_t aw86862_set_pga(uint16_t pga1_gain, uint8_t pga2_gain)
{
    uint8_t u8RegValue;
    int32_t i32Ret;

    i32Ret = aw86862_pga1_gain_to_reg(pga1_gain, &u8RegValue);
    if (LL_OK != i32Ret) {
        return i32Ret;
    }

    if ((pga2_gain < 1U) || (pga2_gain > 8U)) {
        return LL_ERR_INVD_PARAM;
    }

    u8RegValue |= (uint8_t)(pga2_gain - 1U);
    m_stcGainConfig.adch0cr1_0 = u8RegValue;
    return LL_OK;
}
