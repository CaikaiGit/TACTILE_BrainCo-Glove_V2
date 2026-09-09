/**
 *******************************************************************************
 * @file  iap/iap_boot_uart/source/main.c
 * @brief Main program of UART IAP Boot.
 *******************************************************************************
 */

#include "main.h"
#include "com.h"
#include "serial.h"

#include <string.h>

/* unlock/lock peripheral */
#define EXAMPLE_PERIPH_WE               (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
                                         LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM)
#define EXAMPLE_PERIPH_WP               (LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_SRAM)

#define IAP_APP_ADDR                    (FLASH_BASE + IAP_BOOT_SIZE)
#define IAP_PACKET_SIZE                 (2048UL)
#define IAP_CHANNEL_OTA                 (0x0EU)
#define IAP_SIZE_QUERY_BYTE             (0x3CU)
#define IAP_SIZE_QUERY_CHANNEL          (0x02U)
#define IAP_ACK_PAYLOAD_SIZE            (2U)
#define UART_FIRST_BYTE_TIMEOUT_MS      (10UL)
#define UART_INTERBYTE_TIMEOUT_MS       (2UL)
#define IAP_JUMP_DELAY_MS               (500UL)
#define IAP_SIMULATE_UPDATE             (0U)

typedef struct {
    uint16_t u16PacketNum;
    uint16_t u16PacketIndex;
    uint32_t u32LastEraseAddr;
    uint8_t  u8JumpPending;
} stc_iap_state_t;

typedef struct {
    uint16_t u16RxSize;
    uint16_t u16FrameCount;
    uint8_t  au8Head[8];
    uint8_t  u8BootReady;
    uint8_t  u8LastStage;
    uint8_t  u8LastCmd;
    uint8_t  u8RejectCode;
    uint16_t u16LastPacketIndex;
    uint16_t u16LastTxSize;
    uint16_t u16LastLength;
    uint16_t u16LastCrcRecv;
    uint16_t u16LastCrcCalc;
} stc_iap_debug_t;

typedef void (*func_ptr_t)(void);

static uint32_t JumpAddr;
static func_ptr_t JumpToApp;
static serial_t m_stcSerial;
static stc_iap_state_t m_stcIapState;
static volatile stc_iap_debug_t m_stcIapDebug;

static void IAP_CLK_DeInit(void);
static void IAP_PeriphInit(void);
static void IAP_PeriphDeinit(void);
static int32_t IAP_JumpToApp(uint32_t u32Addr);
static void IAP_CheckApp(void);
static void IAP_HandlePacket(package_t *package);
static int32_t IAP_FinalizeUpdate(void);
static int32_t IAP_PrepareAppArea(uint16_t u16PacketNum);
static void IAP_PrepareSizeResponse(uint16_t u16PayloadSize);
static uint16_t IAP_RecvFrame(uint8_t *pu8Buffer, uint16_t u16BufferSize);
static void IAP_ProcessFrame(const uint8_t *pu8Buffer, uint16_t u16Length);

void SysTick_Handler(void)
{
    SysTick_IncTick();
    __DSB();
}

static void SysTick_DeInit(void)
{
    SysTick->CTRL = 0UL;
    SysTick->LOAD = 0UL;
    SysTick->VAL = 0UL;
}

static void IAP_PrepareSizeResponse(uint16_t u16PayloadSize)
{
    m_stcSerial.txbuf[0] = '<';
    m_stcSerial.txbuf[1] = '<';
    m_stcSerial.txbuf[2] = IAP_SIZE_QUERY_CHANNEL;
    m_stcSerial.txbuf[3] = 0x00U;
    m_stcSerial.txbuf[4] = (uint8_t)(u16PayloadSize & 0xFFU);
    m_stcSerial.txbuf[5] = (uint8_t)(u16PayloadSize >> 8U);
    m_stcSerial.txsize = 6U;
}

static int32_t IAP_FinalizeUpdate(void)
{
    uint32_t u32AppAddr = IAP_APP_ADDR;
    uint32_t u32Flag = APP_EXIST_FLAG;

    if (LL_OK != FLASH_EraseSector(APP_UPGRADE_FLAG_ADDR, 0U)) {
        return LL_ERR;
    }
    if (LL_OK != FLASH_WriteData(APP_RUN_ADDR, (uint8_t *)&u32AppAddr, 4U)) {
        return LL_ERR;
    }
    if (LL_OK != FLASH_WriteData(APP_EXIST_FLAG_ADDR, (uint8_t *)&u32Flag, 4U)) {
        return LL_ERR;
    }

    return LL_OK;
}

static int32_t IAP_PrepareAppArea(uint16_t u16PacketNum)
{
    uint32_t u32ImageSize;
    uint32_t u32StartAddr;
    uint32_t u32EndAddr;
    uint32_t u32EraseAddr;

    if (0U == u16PacketNum) {
        return LL_ERR;
    }

    u32ImageSize = (uint32_t)u16PacketNum * IAP_PACKET_SIZE;
    u32StartAddr = IAP_APP_ADDR - (IAP_APP_ADDR % FLASH_SECTOR_SIZE);
    u32EndAddr = IAP_APP_ADDR + u32ImageSize - 1UL;
    u32EndAddr = u32EndAddr - (u32EndAddr % FLASH_SECTOR_SIZE);

    for (u32EraseAddr = u32StartAddr; u32EraseAddr <= u32EndAddr; u32EraseAddr += FLASH_SECTOR_SIZE) {
        if (LL_OK != FLASH_EraseSector(u32EraseAddr, 0U)) {
            return LL_ERR;
        }
    }

    return LL_OK;
}

static void IAP_HandlePacket(package_t *package)
{
    uint8_t u8Cmd;

    if ((NULL == package) || ((package->channel & 0x0FU) != IAP_CHANNEL_OTA)) {
        return;
    }

    m_stcIapDebug.u16LastLength = package->length;
    m_stcIapDebug.u16LastCrcRecv = package->checksum;
    m_stcIapDebug.u16LastCrcCalc = crc_16(package->payload, package->length);
    m_stcIapDebug.u8LastStage = 4U;

    u8Cmd = package_read_u8(package);
    m_stcIapDebug.u8LastCmd = u8Cmd;
    m_stcIapDebug.u8RejectCode = 0U;

    switch (u8Cmd) {
    case 0x00U:
        m_stcIapState.u16PacketNum = package_read_u16(package);
        m_stcIapState.u16PacketIndex = 0U;
        m_stcIapState.u32LastEraseAddr = 0xFFFFFFFFUL;
        m_stcIapState.u8JumpPending = 0U;
#if (IAP_SIMULATE_UPDATE)
        if (0U == m_stcIapState.u16PacketNum) {
            m_stcIapDebug.u8RejectCode = 1U;
            return;
        }
#else
        if ((0U == m_stcIapState.u16PacketNum) ||
            (LL_OK != FLASH_EraseSector(APP_UPGRADE_FLAG_ADDR, 0U)) ||
            (LL_OK != IAP_PrepareAppArea(m_stcIapState.u16PacketNum))) {
            m_stcIapDebug.u8RejectCode = 2U;
            return;
        }
#endif
        m_stcSerial.txsize = pack(m_stcSerial.txbuf,
                                  (uint8_t)(package->channel & 0x0FU),
                                  0x00U,
                                  &m_stcIapState.u16PacketNum,
                                  2U);
        m_stcIapDebug.u16LastTxSize = m_stcSerial.txsize;
        m_stcIapDebug.u8LastStage = 5U;
        break;

    case 0x01U:
        if (package->length >= 3U) {
            uint16_t u16PacketIndex = package_read_u16(package);
            uint16_t u16DataSize = (uint16_t)(package->length - 3U);
#if !IAP_SIMULATE_UPDATE
            uint32_t u32WriteAddr = IAP_APP_ADDR + (((uint32_t)u16PacketIndex - 1UL) * IAP_PACKET_SIZE);
#endif

            if ((0U == m_stcIapState.u16PacketNum) ||
                (u16PacketIndex == 0U) ||
                (u16PacketIndex > m_stcIapState.u16PacketNum) ||
                (u16DataSize > IAP_PACKET_SIZE)) {
                m_stcIapDebug.u16LastPacketIndex = u16PacketIndex;
                m_stcIapDebug.u8RejectCode = 3U;
                return;
            }

#if !IAP_SIMULATE_UPDATE
            if (LL_OK != FLASH_WriteData(u32WriteAddr, &package->payload[3], u16DataSize)) {
                m_stcIapDebug.u16LastPacketIndex = u16PacketIndex;
                m_stcIapDebug.u8RejectCode = 4U;
                return;
            }
#endif

            m_stcIapState.u16PacketIndex = u16PacketIndex;
            m_stcIapDebug.u16LastPacketIndex = u16PacketIndex;

#if !IAP_SIMULATE_UPDATE
            if ((u16PacketIndex == m_stcIapState.u16PacketNum) &&
                (LL_OK != IAP_FinalizeUpdate())) {
                m_stcIapDebug.u8RejectCode = 5U;
                return;
            }
#endif

            m_stcSerial.txsize = pack(m_stcSerial.txbuf,
                                      (uint8_t)(package->channel & 0x0FU),
                                      0x00U,
                                      &u16PacketIndex,
                                      2U);
            m_stcIapDebug.u16LastTxSize = m_stcSerial.txsize;
            if (u16PacketIndex == m_stcIapState.u16PacketNum) {
#if !IAP_SIMULATE_UPDATE
                m_stcIapState.u8JumpPending = 1U;
#endif
            }
            m_stcIapDebug.u8LastStage = 6U;
        } else {
            m_stcIapDebug.u8RejectCode = 6U;
        }
        break;

    default:
        break;
    }
}

static void IAP_ProcessFrame(const uint8_t *pu8Buffer, uint16_t u16Length)
{
    uint16_t u16CopyLen;

    if ((NULL == pu8Buffer) || (0U == u16Length)) {
        return;
    }

    m_stcIapDebug.u16RxSize = u16Length;
    (void)memset((void *)m_stcIapDebug.au8Head, 0, sizeof(m_stcIapDebug.au8Head));
    u16CopyLen = (u16Length < sizeof(m_stcIapDebug.au8Head)) ? u16Length : (uint16_t)sizeof(m_stcIapDebug.au8Head);
    (void)memcpy((void *)m_stcIapDebug.au8Head, pu8Buffer, u16CopyLen);

    if ((1U == u16Length) && (pu8Buffer[0] == IAP_SIZE_QUERY_BYTE)) {
        IAP_PrepareSizeResponse(IAP_ACK_PAYLOAD_SIZE);
        COM_SendData(m_stcSerial.txbuf, m_stcSerial.txsize);
        m_stcIapDebug.u8LastStage = 2U;
        return;
    }

    if (u16Length >= 10U) {
        m_stcIapDebug.u16FrameCount++;
        m_stcIapDebug.u8LastStage = 3U;
        (void)memset(&m_stcSerial.package, 0, sizeof(m_stcSerial.package));
        m_stcSerial.rxsize = u16Length;
        (void)memcpy(m_stcSerial.rxbuf, pu8Buffer, u16Length);
        unpack(&m_stcSerial);
        if (m_stcSerial.txsize > 0U) {
            COM_SendData(m_stcSerial.txbuf, m_stcSerial.txsize);
            if (0U != m_stcIapState.u8JumpPending) {
                DDL_DelayMS(IAP_JUMP_DELAY_MS);
                m_stcIapState.u8JumpPending = 0U;
                (void)IAP_JumpToApp(IAP_APP_ADDR);
            }
        }
    }
}

static uint16_t IAP_RecvFrame(uint8_t *pu8Buffer, uint16_t u16BufferSize)
{
    uint8_t u8Data;
    uint16_t u16Length = 0U;

    if ((NULL == pu8Buffer) || (0U == u16BufferSize)) {
        return 0U;
    }

    if (LL_OK != COM_RecvData(&u8Data, 1U, UART_FIRST_BYTE_TIMEOUT_MS)) {
        return 0U;
    }

    pu8Buffer[u16Length++] = u8Data;
    while (u16Length < u16BufferSize) {
        if (LL_OK != COM_RecvData(&u8Data, 1U, UART_INTERBYTE_TIMEOUT_MS)) {
            break;
        }
        pu8Buffer[u16Length++] = u8Data;
    }

    return u16Length;
}

static void IAP_CLK_DeInit(void)
{
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_MRC);
    (void)PWC_HighPerformanceToHighSpeed();
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV1 | CLK_PCLK0_DIV1 |
                                      CLK_PCLK1_DIV1 | CLK_PCLK2_DIV1 | CLK_PCLK3_DIV1 |
                                      CLK_PCLK4_DIV1));
    (void)CLK_HrcCmd(ENABLE);
}

static void IAP_PeriphInit(void)
{
    LL_PERIPH_WE(EXAMPLE_PERIPH_WE);
    BSP_CLK_Init();
    SysTick_Init(1000U);

    (void)memset(&m_stcIapState, 0, sizeof(m_stcIapState));
    (void)memset((void *)&m_stcIapDebug, 0, sizeof(m_stcIapDebug));
    (void)memset(&m_stcSerial, 0, sizeof(m_stcSerial));
    m_stcIapState.u32LastEraseAddr = 0xFFFFFFFFUL;
    m_stcSerial.onready = &IAP_HandlePacket;

    COM_Init();
    IAP_PrepareSizeResponse(IAP_ACK_PAYLOAD_SIZE);
    m_stcIapDebug.u8BootReady = 1U;
}

static void IAP_PeriphDeinit(void)
{
    COM_DeInit();
    SysTick_DeInit();
    IAP_CLK_DeInit();
    LL_PERIPH_WP(EXAMPLE_PERIPH_WP);
}

static int32_t IAP_JumpToApp(uint32_t u32Addr)
{
    uint32_t u32StackTop = *((__IO uint32_t *)u32Addr);
    uint32_t i;

    if ((u32StackTop > SRAM_BASE) && (u32StackTop <= (SRAM_BASE + SRAM_SIZE))) {
        __disable_irq();
        for (i = 0UL; i < 8UL; ++i) {
            NVIC->ICER[i] = 0xFFFFFFFFUL;
            NVIC->ICPR[i] = 0xFFFFFFFFUL;
        }
        IAP_PeriphDeinit();
        SCB->VTOR = u32Addr;
        __DSB();
        __ISB();
        JumpAddr = *(__IO uint32_t *)(u32Addr + 4U);
        JumpToApp = (func_ptr_t)JumpAddr;
        __set_MSP(u32StackTop);
        __enable_irq();
        JumpToApp();
    }

    return LL_ERR;
}

static void IAP_CheckApp(void)
{
    uint32_t u32UpgradeFlag;
    uint32_t u32ExistFlag;
    uint32_t u32RunAddr;

    if (LL_OK != FLASH_ReadData(APP_UPGRADE_FLAG_ADDR, (uint8_t *)&u32UpgradeFlag, 4U)) {
        return;
    }
    if (APP_UPGRADE_FLAG == u32UpgradeFlag) {
        return;
    }

    if ((LL_OK == FLASH_ReadData(APP_EXIST_FLAG_ADDR, (uint8_t *)&u32ExistFlag, 4U)) &&
        (LL_OK == FLASH_ReadData(APP_RUN_ADDR, (uint8_t *)&u32RunAddr, 4U)) &&
        (APP_EXIST_FLAG == u32ExistFlag) &&
        (u32RunAddr >= IAP_APP_ADDR) &&
        (u32RunAddr < (FLASH_BASE + FLASH_SIZE))) {
        (void)IAP_JumpToApp(u32RunAddr);
    }
}

int main(void)
{
    uint16_t u16Length;

    IAP_PeriphInit();
    IAP_CheckApp();

    for (;;) {
        u16Length = IAP_RecvFrame(m_stcSerial.rxbuf, (uint16_t)sizeof(m_stcSerial.rxbuf));
        if (u16Length > 0U) {
            IAP_ProcessFrame(m_stcSerial.rxbuf, u16Length);
        }
    }
}
