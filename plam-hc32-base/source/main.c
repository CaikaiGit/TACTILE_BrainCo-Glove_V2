/**
 *******************************************************************************
 * @file  usart/usart_uart_dma/source/main.c
 * @brief This example demonstrates UART data receive and transfer by DMA.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2022-10-31       CDT             Delete the redundant code
                                    Read USART_DR.RDR when USART overrun error occur.
   2023-01-15       CDT             Update UART timeout function calculating formula for Timer0 CMP value
   2023-09-30       CDT             Split register USART_DR to USART_RDR and USART_TDR
   2024-11-08       CDT             Optimize function: USART_TxComplete_IrqCallback
                                    Add function: USART_StopTimeoutTimer
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
#include "app_base.h"
#include "sampler.h"
#include "serial.h"
#include "system.h"
#include "time.h"
#include <stdio.h>
#include <string.h>

/**
 * @addtogroup HC32F460_DDL_Examples
 * @{
 */

/**
 * @addtogroup USART_UART_DMA
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Peripheral register WE/WP selection */
#define LL_PERIPH_SEL                   (LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | \
                                         LL_PERIPH_EFM | LL_PERIPH_SRAM)

/* DMA definition */
#define RX_DMA_UNIT                     (CM_DMA1)
#define RX_DMA_CH                       (DMA_CH0)
#define RX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE))
#define RX_DMA_TRIG_SEL                 (AOS_DMA1_0)
#define RX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART1_RI)
#define RX_DMA_RECONF_TRIG_SEL          (AOS_DMA_RC)
#define RX_DMA_RECONF_TRIG_EVT_SRC      (EVT_SRC_AOS_STRG)
#define RX_DMA_TC_INT                   (DMA_INT_TC_CH0)
#define RX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH0)
#define RX_DMA_TC_IRQn                  (INT000_IRQn)
#define RX_DMA_TC_INT_SRC               (INT_SRC_DMA1_TC0)

#define TX_DMA_UNIT                     (CM_DMA2)
#define TX_DMA_CH                       (DMA_CH0)
#define TX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define TX_DMA_TRIG_SEL                 (AOS_DMA2_0)
#define TX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART1_TI)
#define TX_DMA_TC_INT                   (DMA_INT_TC_CH0)
#define TX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH0)
#define TX_DMA_TC_IRQn                  (INT001_IRQn)
#define TX_DMA_TC_INT_SRC               (INT_SRC_DMA2_TC0)

/* Timer0 unit & channel definition */
#define TMR0_UNIT                       (CM_TMR0_2)
#define TMR0_CH                         (TMR0_CH_B)
#define TMR0_FCG_ENABLE()               (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_2, ENABLE))
#define RX_POLL_TMR0_CH                 (TMR0_CH_A)
#define RX_POLL_TMR0_INT                (TMR0_INT_CMP_A)
#define RX_POLL_TMR0_FLAG               (TMR0_FLAG_CMP_A)
#define RX_POLL_TMR0_INT_SRC            (INT_SRC_TMR0_2_CMP_A)
#define RX_POLL_TMR0_IRQn               (INT005_IRQn)
#define RX_POLL_TMR0_PERIOD_US          (20U)
#define RX_POLL_IDLE_TICKS              (2U)

/* USART RX/TX pin definition */
#define USART_RX_PORT                   (GPIO_PORT_A)   /* PA11: USART1_RX */
#define USART_RX_PIN                    (GPIO_PIN_11)
#define USART_RX_GPIO_FUNC              (GPIO_FUNC_33)

#define USART_TX_PORT                   (GPIO_PORT_A)   /* PA12: USART1_TX */
#define USART_TX_PIN                    (GPIO_PIN_12)
#define USART_TX_GPIO_FUNC              (GPIO_FUNC_32)

/* USART unit definition */
#define USART_UNIT                      (CM_USART1)
#define USART_FCG_ENABLE()              (FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART1, ENABLE))

/* USART baudrate definition */
#define USART_BAUDRATE                  (2000000UL)

/* USART timeout bits definition */
#define USART_TIMEOUT_BITS              (256U)
#define APP_RX_IDLE_TIMEOUT_US          (50U)

/* USART interrupt definition */
#define USART_TX_CPLT_IRQn              (INT002_IRQn)
#define USART_TX_CPLT_INT_SRC           (INT_SRC_USART1_TCI)

#define USART_RX_ERR_IRQn               (INT003_IRQn)
#define USART_RX_ERR_INT_SRC            (INT_SRC_USART1_EI)

#define USART_RX_TIMEOUT_IRQn           (INT004_IRQn)
#define USART_RX_TIMEOUT_INT_SRC        (INT_SRC_USART1_RTO)

/* Application frame length max definition */
#define APP_FRAME_LEN_MAX               (SERIAL_RX_BUFFER_SIZE)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static __IO en_flag_status_t m_enRxFrameEnd;
static __IO uint16_t m_u16RxLen;
static uint8_t *m_pu8RxBuf;
static uint16_t m_u16LastRxCount;
static uint32_t m_u32LastRxTickUs;
static uint8_t m_u8RxStableTicks;

static void App_HardwareInit(void)
{
    sampler_hardware_init();
}

static void App_FaultLoop(void)
{
    for (;;) {
    }
}

static void App_ClockInit(void)
{
    stc_clock_pll_init_t stcMpllInit;

    (void)CLK_PLLStructInit(&stcMpllInit);

    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 |
                                      CLK_EXCLK_DIV2 |
                                      CLK_PCLK0_DIV1 |
                                      CLK_PCLK1_DIV2 |
                                      CLK_PCLK2_DIV4 |
                                      CLK_PCLK3_DIV4 |
                                      CLK_PCLK4_DIV2));

    (void)CLK_HrcCmd(ENABLE);

    stcMpllInit.PLLCFGR = 0UL;
    stcMpllInit.PLLCFGR_f.PLLM = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLN = 50UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLP = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLQ = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLR = 2UL - 1UL;
    stcMpllInit.u8PLLState = CLK_PLL_ON;
    stcMpllInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_HRC;
    (void)CLK_PLLInit(&stcMpllInit);

    while (SET != CLK_GetStableStatus(CLK_STB_FLAG_PLL)) {
    }

    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    SRAM_SetWaitCycle((SRAM_SRAM12 | SRAM_SRAM3 | SRAM_SRAMR), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);
    (void)PWC_HighSpeedToHighPerformance();
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
    EFM_CacheRamReset(ENABLE);
    EFM_CacheRamReset(DISABLE);
    EFM_CacheCmd(ENABLE);
}

static uint32_t App_GetTick(void)
{
    return SysTick_GetTick();
}

static uint32_t App_GetTickUs(void)
{
    uint32_t u32Ms1;
    uint32_t u32Ms2;
    uint32_t u32Load;
    uint32_t u32Val;
    uint32_t u32ElapsedUs;

    u32Load = SysTick->LOAD + 1UL;
    do {
        u32Ms1 = SysTick_GetTick();
        u32Val = SysTick->VAL;
        u32Ms2 = SysTick_GetTick();
    } while (u32Ms1 != u32Ms2);

    u32ElapsedUs = ((u32Load - u32Val) * 1000UL) / u32Load;
    return (u32Ms1 * 1000UL) + u32ElapsedUs;
}

static void App_DelayMs(uint32_t ms)
{
    DDL_DelayMS(ms);
}

static void App_DelayUs(uint32_t us)
{
    DDL_DelayUS(us);
}

static int32_t App_SerialWrite(void *ctx, const uint8_t *data, uint16_t size)
{
    uint16_t i;
    CM_USART_TypeDef *USARTx = (CM_USART_TypeDef *)ctx;

    if ((USARTx == NULL) || (data == NULL)) {
        return -1;
    }

    USART_FuncCmd(USARTx, USART_TX, ENABLE);
    for (i = 0U; i < size; ++i) {
        while (RESET == USART_GetStatus(USARTx, USART_FLAG_TX_EMPTY)) {
        }
        USART_WriteData(USARTx, data[i]);
    }
    while (RESET == USART_GetStatus(USARTx, USART_FLAG_TX_CPLT)) {
    }

    return size;
}

static int32_t App_SerialWriteDma(void *ctx, const uint8_t *data, uint16_t size)
{
    CM_USART_TypeDef *USARTx = (CM_USART_TypeDef *)ctx;

    if ((USARTx == NULL) || (data == NULL) || (size == 0U) || (size > SERIAL_TX_BUFFER_SIZE)) {
        return -1;
    }

    (void)DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, TX_DMA_TC_FLAG);
    (void)DMA_SetSrcAddr(TX_DMA_UNIT, TX_DMA_CH, (uint32_t)data);
    (void)DMA_SetTransCount(TX_DMA_UNIT, TX_DMA_CH, size);
    (void)DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, ENABLE);

    USART_FuncCmd(USARTx, USART_TX, ENABLE);
    return size;
}

static void App_WriteString(const char *text)
{
    if (text != NULL) {
        (void)App_SerialWrite(USART_UNIT, (const uint8_t *)text, (uint16_t)strlen(text));
    }
}

static void App_SerialStartRx(void *ctx, uint8_t *buffer, uint16_t size)
{
    (void)ctx;
    (void)size;
    m_pu8RxBuf = buffer;
    m_u16LastRxCount = 0U;
    m_u32LastRxTickUs = 0UL;
    m_u8RxStableTicks = 0U;
}

static void App_PollRxFrameEnd(void)
{
    uint16_t u16RxCount;

    if (m_enRxFrameEnd == SET) {
        return;
    }

    u16RxCount = APP_FRAME_LEN_MAX - (uint16_t)DMA_GetTransCount(RX_DMA_UNIT, RX_DMA_CH);
    if (u16RxCount == 0U) {
        m_u16LastRxCount = 0U;
        m_u8RxStableTicks = 0U;
        return;
    }

    if (u16RxCount != m_u16LastRxCount) {
        m_u16LastRxCount = u16RxCount;
        m_u32LastRxTickUs = App_GetTickUs();
        m_u8RxStableTicks = 0U;
        return;
    }

    if (m_u8RxStableTicks < 0xFFU) {
        m_u8RxStableTicks++;
    }

    if ((m_u8RxStableTicks >= RX_POLL_IDLE_TICKS) ||
        ((App_GetTickUs() - m_u32LastRxTickUs) >= APP_RX_IDLE_TIMEOUT_US)) {
        m_enRxFrameEnd = SET;
        m_u16RxLen = u16RxCount;
        m_u16LastRxCount = 0U;
        m_u8RxStableTicks = 0U;
        AOS_SW_Trigger();
    }
}

static void App_ServiceRxFrameEnd(void)
{
    App_PollRxFrameEnd();
    if (SET == m_enRxFrameEnd) {
        system_serial_rx_notify(m_u16RxLen);
        m_enRxFrameEnd = RESET;
        m_u16RxLen = 0U;
    }
}

static void RX_POLL_TMR0_IrqCallback(void)
{
    App_ServiceRxFrameEnd();
    TMR0_ClearStatus(TMR0_UNIT, RX_POLL_TMR0_FLAG);
}

void SysTick_Handler(void)
{
    SysTick_IncTick();
}

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 * @brief  DMA transfer complete IRQ callback function.
 * @param  None
 * @retval None
 */
static void RX_DMA_TC_IrqCallback(void)
{
    m_enRxFrameEnd = SET;
    m_u16RxLen = APP_FRAME_LEN_MAX;
    system_serial_rx_notify(m_u16RxLen);
    m_enRxFrameEnd = RESET;
    m_u16RxLen = 0U;

    USART_FuncCmd(USART_UNIT, USART_RX_TIMEOUT, DISABLE);

    DMA_ClearTransCompleteStatus(RX_DMA_UNIT, RX_DMA_TC_FLAG);
}

/**
 * @brief  DMA transfer complete IRQ callback function.
 * @param  None
 * @retval None
 */
static void TX_DMA_TC_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, USART_INT_TX_CPLT, ENABLE);

    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, TX_DMA_TC_FLAG);
}

/**
 * @brief  Initialize DMA.
 * @param  None
 * @retval int32_t:
 *           - LL_OK:                   Initialize successfully.
 *           - LL_ERR_INVD_PARAM:       Initialization parameters is invalid.
 */
static int32_t DMA_Config(void)
{
    int32_t i32Ret;
    stc_dma_init_t stcDmaInit;
    stc_dma_llp_init_t stcDmaLlpInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    static stc_dma_llp_descriptor_t stcLlpDesc;

    /* DMA&AOS FCG enable */
    RX_DMA_FCG_ENABLE();
    TX_DMA_FCG_ENABLE();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* USART_RX_DMA */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = APP_FRAME_LEN_MAX;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr = (uint32_t)m_pu8RxBuf;
    stcDmaInit.u32SrcAddr = (uint32_t)(&USART_UNIT->RDR);
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
    i32Ret = DMA_Init(RX_DMA_UNIT, RX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        (void)DMA_LlpStructInit(&stcDmaLlpInit);
        stcDmaLlpInit.u32State = DMA_LLP_ENABLE;
        stcDmaLlpInit.u32Mode  = DMA_LLP_WAIT;
        stcDmaLlpInit.u32Addr  = (uint32_t)&stcLlpDesc;
        (void)DMA_LlpInit(RX_DMA_UNIT, RX_DMA_CH, &stcDmaLlpInit);

        stcLlpDesc.SARx   = stcDmaInit.u32SrcAddr;
        stcLlpDesc.DARx   = stcDmaInit.u32DestAddr;
        stcLlpDesc.DTCTLx = (stcDmaInit.u32TransCount << DMA_DTCTL_CNT_POS) | (stcDmaInit.u32BlockSize << DMA_DTCTL_BLKSIZE_POS);;
        stcLlpDesc.LLPx   = (uint32_t)&stcLlpDesc;
        stcLlpDesc.CHCTLx = stcDmaInit.u32SrcAddrInc | stcDmaInit.u32DestAddrInc | stcDmaInit.u32DataWidth |  \
                            stcDmaInit.u32IntEn      | stcDmaLlpInit.u32State    | stcDmaLlpInit.u32Mode;

        DMA_ReconfigLlpCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
        DMA_ReconfigCmd(RX_DMA_UNIT, ENABLE);
        AOS_SetTriggerEventSrc(RX_DMA_RECONF_TRIG_SEL, RX_DMA_RECONF_TRIG_EVT_SRC);

        stcIrqSignConfig.enIntSrc = RX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn  = RX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &RX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(RX_DMA_TRIG_SEL, RX_DMA_TRIG_EVT_SRC);

        DMA_Cmd(RX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(RX_DMA_UNIT, RX_DMA_TC_INT, ENABLE);
        (void)DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
    }

    /* USART_TX_DMA */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = APP_FRAME_LEN_MAX;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr = (uint32_t)(&USART_UNIT->TDR);
    stcDmaInit.u32SrcAddr = (uint32_t)m_pu8RxBuf;
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    i32Ret = DMA_Init(TX_DMA_UNIT, TX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        stcIrqSignConfig.enIntSrc = TX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn  = TX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &TX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(TX_DMA_TRIG_SEL, TX_DMA_TRIG_EVT_SRC);

        DMA_Cmd(TX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(TX_DMA_UNIT, TX_DMA_TC_INT, ENABLE);
    }

    return i32Ret;
}

/**
 * @brief  Configure TMR0.
 * @param  [in] u16TimeoutBits:         Timeout bits
 * @retval None
 */
static void TMR0_Config(uint16_t u16TimeoutBits)
{
    uint16_t u16Div;
    uint16_t u16Delay;
    uint16_t u16CompareValue;
    uint16_t u16PollCompareValue;
    stc_tmr0_init_t stcTmr0Init;
    stc_irq_signin_config_t stcIrqSignConfig;

    TMR0_FCG_ENABLE();

    /* Initialize TMR0 base function. */
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_INTERN_CLK;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV8;
    stcTmr0Init.u32Func     = TMR0_FUNC_CMP;
    if (TMR0_CLK_DIV1 == stcTmr0Init.u32ClockDiv) {
        u16Delay = 7U;
    } else if (TMR0_CLK_DIV2 == stcTmr0Init.u32ClockDiv) {
        u16Delay = 5U;
    } else if ((TMR0_CLK_DIV4 == stcTmr0Init.u32ClockDiv) || \
               (TMR0_CLK_DIV8 == stcTmr0Init.u32ClockDiv) || \
               (TMR0_CLK_DIV16 == stcTmr0Init.u32ClockDiv)) {
        u16Delay = 3U;
    } else {
        u16Delay = 2U;
    }

    u16Div = (uint16_t)1U << (stcTmr0Init.u32ClockDiv >> TMR0_BCONR_CKDIVA_POS);
    u16CompareValue = ((u16TimeoutBits + u16Div - 1U) / u16Div) - u16Delay;
    stcTmr0Init.u16CompareValue = u16CompareValue;
    (void)TMR0_Init(TMR0_UNIT, TMR0_CH, &stcTmr0Init);

    TMR0_HWStartCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);
    TMR0_HWClearCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);

    (void)TMR0_StructInit(&stcTmr0Init);
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_INTERN_CLK;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV8;
    stcTmr0Init.u32Func     = TMR0_FUNC_CMP;
    u16PollCompareValue = (uint16_t)(((SystemCoreClock / 8UL) * RX_POLL_TMR0_PERIOD_US) / 1000000UL);
    if (u16PollCompareValue == 0U) {
        u16PollCompareValue = 1U;
    }
    stcTmr0Init.u16CompareValue = (uint16_t)(u16PollCompareValue - 1U);
    (void)TMR0_Init(TMR0_UNIT, RX_POLL_TMR0_CH, &stcTmr0Init);
    TMR0_IntCmd(TMR0_UNIT, RX_POLL_TMR0_INT, ENABLE);

    stcIrqSignConfig.enIntSrc    = RX_POLL_TMR0_INT_SRC;
    stcIrqSignConfig.enIRQn      = RX_POLL_TMR0_IRQn;
    stcIrqSignConfig.pfnCallback = &RX_POLL_TMR0_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
    TMR0_Start(TMR0_UNIT, RX_POLL_TMR0_CH);
}

/**
 * @brief  Stop timeout timer.
 * @param  [in]  TMR0x                  Pointer to TMR0 instance register base.
 *                                      This parameter can be a value of the following:
 *   @arg  CM_TMR0_x or CM_TMR0
 * @param  [in]  u32Ch                  TMR0 channel.
 *                                      This parameter can be a value @ref TMR0_Channel
 */
static void USART_StopTimeoutTimer(CM_TMR0_TypeDef *TMR0x, uint32_t u32Ch)
{
    uint32_t u32ClrMask;
    uint32_t u32SetMask;
    uint32_t u32BitOffset;

    u32BitOffset = 16UL * u32Ch;

    /* Set: TMR0_BCONR.SYNCLKA<B>=1, TMR0_BCONR.SYNA<B>=0 */
    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA) << u32BitOffset;
    u32SetMask = TMR0_BCONR_SYNCLKA << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);

    /* Set: TMR0_BCONR.CSTA<B>=0, TMR0_BCONR.SYNCLKA<B>=0, TMR0_BCONR.SYNSA<B>=1 */
    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA | TMR0_BCONR_CSTA) << u32BitOffset;
    u32SetMask = TMR0_BCONR_SYNSA << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);
}

/**
 * @brief  USART RX timeout IRQ callback.
 * @param  None
 * @retval None
 */
static void USART_RxTimeout_IrqCallback(void)
{
    if (m_enRxFrameEnd != SET) {
        m_enRxFrameEnd = SET;
        m_u16RxLen = APP_FRAME_LEN_MAX - (uint16_t)DMA_GetTransCount(RX_DMA_UNIT, RX_DMA_CH);
        system_serial_rx_notify(m_u16RxLen);
        m_enRxFrameEnd = RESET;
        m_u16RxLen = 0U;

        /* Trigger for re-config USART RX DMA */
        AOS_SW_Trigger();
    }

    USART_StopTimeoutTimer(TMR0_UNIT, TMR0_CH);

    USART_ClearStatus(USART_UNIT, USART_FLAG_RX_TIMEOUT);
}

/**
 * @brief  USART TX complete IRQ callback function.
 * @param  None
 * @retval None
 */
static void USART_TxComplete_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    system_serial_tx_complete();
}

/**
 * @brief  USART RX error IRQ callback.
 * @param  None
 * @retval None
 */
static void USART_RxError_IrqCallback(void)
{
    (void)USART_ReadData(USART_UNIT);

    USART_ClearStatus(USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

void system_prepare_for_reset(void)
{
    __disable_irq();

    USART_FuncCmd(USART_UNIT, (USART_RX | USART_TX | USART_INT_RX | USART_INT_TX_CPLT |
                               USART_RX_TIMEOUT | USART_INT_RX_TIMEOUT), DISABLE);
    USART_ClearStatus(USART_UNIT, (USART_FLAG_RX_TIMEOUT | USART_FLAG_PARITY_ERR |
                                   USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));

    (void)DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, DISABLE);
    (void)DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    DMA_TransCompleteIntCmd(RX_DMA_UNIT, RX_DMA_TC_INT, DISABLE);
    DMA_TransCompleteIntCmd(TX_DMA_UNIT, TX_DMA_TC_INT, DISABLE);
    DMA_Cmd(RX_DMA_UNIT, DISABLE);
    DMA_Cmd(TX_DMA_UNIT, DISABLE);

    TMR0_DeInit(TMR0_UNIT);
    USART_DeInit(USART_UNIT);

    NVIC_DisableIRQ(RX_DMA_TC_IRQn);
    NVIC_DisableIRQ(TX_DMA_TC_IRQn);
    NVIC_DisableIRQ(USART_TX_CPLT_IRQn);
    NVIC_DisableIRQ(USART_RX_ERR_IRQn);
    NVIC_DisableIRQ(USART_RX_TIMEOUT_IRQn);
    NVIC_DisableIRQ(RX_POLL_TMR0_IRQn);
    NVIC_ClearPendingIRQ(RX_DMA_TC_IRQn);
    NVIC_ClearPendingIRQ(TX_DMA_TC_IRQn);
    NVIC_ClearPendingIRQ(USART_TX_CPLT_IRQn);
    NVIC_ClearPendingIRQ(USART_RX_ERR_IRQn);
    NVIC_ClearPendingIRQ(USART_RX_TIMEOUT_IRQn);
    NVIC_ClearPendingIRQ(RX_POLL_TMR0_IRQn);

    m_enRxFrameEnd = RESET;
    m_u16RxLen = 0U;
    m_u16LastRxCount = 0U;
    m_u32LastRxTickUs = 0UL;
    m_u8RxStableTicks = 0U;
}

/**
 * @brief  Main function of UART DMA project
 * @param  None
 * @retval int32_t return value, if needed
 */
int32_t main(void)
{
    stc_usart_uart_init_t stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;
    system_port_t stcSystemPort;
    int32_t i32Ret;

    /* MCU Peripheral registers write unprotected */
    LL_PERIPH_WE(LL_PERIPH_SEL);

    /* Initialize system clock from internal HRC. */
    App_ClockInit();
    SCB->VTOR = APP_CODE_BASE;
    __DSB();
    __ISB();
    (void)SysTick_Init(1000U);
    time_set_delay_us_callback(&App_DelayUs);

    m_pu8RxBuf = system_get_serial_rx_buffer();
    if ((m_pu8RxBuf == NULL) || (system_get_serial_rx_buffer_size() < APP_FRAME_LEN_MAX)) {
        App_FaultLoop();
    }

    /* Initialize DMA and USART timeout timer before system_init() starts RX. */
    (void)DMA_Config();
    TMR0_Config(USART_TIMEOUT_BITS);

    /* Configure USART RX/TX pin. */
    GPIO_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_GPIO_FUNC);
    GPIO_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_GPIO_FUNC);

    /* Enable peripheral clock */
    USART_FCG_ENABLE();

    /* Initialize UART. */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV4;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = USART_BAUDRATE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(USART_UNIT, &stcUartInit, NULL)) {
        App_FaultLoop();
    }

    stcIrqSigninConfig.enIRQn = USART_TX_CPLT_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_TX_CPLT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_TxComplete_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    stcIrqSigninConfig.enIRQn = USART_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxError_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    stcIrqSigninConfig.enIRQn = USART_RX_TIMEOUT_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_RX_TIMEOUT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxTimeout_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    (void)memset(&stcSystemPort, 0, sizeof(stcSystemPort));
    stcSystemPort.serial_ctx = USART_UNIT;
    stcSystemPort.get_tick = &App_GetTick;
    stcSystemPort.delay_ms = &App_DelayMs;
    stcSystemPort.write = &App_SerialWrite;
    stcSystemPort.write_dma = &App_SerialWriteDma;
    stcSystemPort.toggle_tx = NULL;
    stcSystemPort.start_rx = &App_SerialStartRx;
    stcSystemPort.hardware_init = &App_HardwareInit;
    system_set_port(&stcSystemPort);
    system_init();

    /* MCU Peripheral registers write protected */
    LL_PERIPH_WP(LL_PERIPH_SEL);

    USART_FuncCmd(USART_UNIT, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT |
                               USART_INT_RX_TIMEOUT), ENABLE);

    for (;;) {
        system_run();
    }
}

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
