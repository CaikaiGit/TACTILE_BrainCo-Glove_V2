#include "configuration.h"

#include "app_base.h"

#include "calibration.h"
#include "force_lut_storage.h"
#include "internal_flash.h"
#include "serial.h"
#include "system.h"
#include "version.h"

#include <stdio.h>
#include <string.h>

extern serial_t serial1;
extern uint8_t  filter_enabled;
extern volatile uint8_t  package_id;
extern config_t config_;
extern bool press_flag; // pressure mode flag: false uploads AD value, true uploads pressure value
#if (PRESS_CALIBRATION_AREA_COUNT > 1U)
static uint8_t press_lut_flag[4] = { 0U };
#endif

#define FORCE_LUT_RX_META         0x01U
#define FORCE_LUT_RX_AREA_CX      0x02U
#define FORCE_LUT_RX_AREA_CY      0x04U
#define FORCE_LUT_RX_FORCE_GRID   0x08U
#define FORCE_LUT_RX_SENSOR_TABLE 0x10U
#define FORCE_LUT_RX_READY        (FORCE_LUT_RX_META | FORCE_LUT_RX_AREA_CX | \
                                   FORCE_LUT_RX_AREA_CY | FORCE_LUT_RX_FORCE_GRID | \
                                   FORCE_LUT_RX_SENSOR_TABLE)
#define FORCE_LUT_ALL_PARTS_MASK  ((uint8_t)((1UL << FORCE_LUT_PART_COUNT) - 1UL))

static force_lut_part_data_t g_force_lut_rx;
static uint8_t               g_force_lut_rx_flags;
static uint8_t               g_force_lut_written_mask;
enum
{
    ATTRIBUTE = 0x01,
    DATA      = 0x02,
    ZERO      = 0x07,
    CONFIG    = 0X09,
    OTA       = 0x0e
};

enum
{
    CONFIG_CALIBRATION             = 0x01,
    CONFIG_ZEROING                 = 0x02,
    CONFIG_CALIBRATION_COEFFICIENT_1 = 0x03,
#if (PRESS_CALIBRATION_AREA_COUNT > 1U)
	CONFIG_CALIBRATION_COEFFICIENT_2 = 0x23,
	CONFIG_CALIBRATION_COEFFICIENT_3 = 0x33,
	CONFIG_CALIBRATION_COEFFICIENT_4 = 0x43,
#endif
};
enum
{
    CONFIG_MULTIPLE_SERIAL = 0x01,
    CONFIG_FILTER          = 0x02,
    CONFIG_BYTES_NUM       = 0x03,
    CONFIG_ADDRESS         = 0x04
};
void retrieve_chip_id(uint8_t *uid);

#define IAP_BOOT_SIZE           (0x8000UL)
#define APP_UPGRADE_FLAG_ADDR   (EFM_BASE + IAP_BOOT_SIZE - 8UL)
#define APP_UPGRADE_FLAG        (0xA5B6C7D8UL)

static void request_ota_upgrade(void)
{
#if (APP_CODE_BASE != 0UL)
    uint32_t u32Primask;
    uint32_t u32Flag = APP_UPGRADE_FLAG;

    u32Primask = __get_PRIMASK();
    __disable_irq();
    while (SET != EFM_GetStatus(EFM_FLAG_RDY)) {
    }
    EFM_REG_Unlock();
    EFM_FWMC_Cmd(ENABLE);
    (void)EFM_SectorErase(APP_UPGRADE_FLAG_ADDR);
    (void)EFM_Program(APP_UPGRADE_FLAG_ADDR, (uint8_t *)&u32Flag, 4U);
    while (SET != EFM_GetStatus(EFM_FLAG_RDY)) {
    }
    EFM_FWMC_Cmd(DISABLE);
    if (0U == u32Primask) {
        __enable_irq();
    }
#endif
}

static uint16_t force_lut_float_to_u16(float value)
{
    if (value <= 0.0f) {
        return 0U;
    }
    if (value >= 65535.0f) {
        return 65535U;
    }
    return (uint16_t)(value + 0.5f);
}

static uint8_t force_lut_read_float_array(package_t *package, float *out, uint8_t count, uint16_t max_count)
{
    uint8_t i;

    if ((out == NULL) || ((uint16_t)count > max_count) ||
        (package_remain_size(package) < ((uint16_t)count * sizeof(float)))) {
        return 0U;
    }

    for (i = 0U; i < count; ++i) {
        out[i] = package_read_f32(package);
    }

    return 1U;
}

static uint8_t force_lut_part_mask(uint8_t part_id)
{
    int index = force_lut_part_index(part_id);

    if (index < 0) {
        return 0U;
    }

    return (uint8_t)(1UL << (uint8_t)index);
}

static uint8_t pressure_mode_write_flag(uint8_t enabled)
{
    uint8_t  flag = (enabled != 0U) ? 1U : 0U;
    uint8_t  ok = 0U;
    uint32_t primask;

    press_flag = (flag != 0U);
    primask = __get_PRIMASK();
    __disable_irq();
    if (FLASH_OK == flash_init()) {
        if (FLASH_OK == flash_pack_and_write(ADDR_FLASH_SECTOR_PRESS_MODE_FLAG, &flag, 1U)) {
            ok = 1U;
        }
        (void)flash_deinit();
    }
    if (0U == primask) {
        __enable_irq();
    }

    return ok;
}

static uint8_t force_lut_write_ready_part(void)
{
    uint8_t  write_ok = 0U;
    uint8_t  part_mask;
    uint8_t  loaded_count;
    uint32_t primask;

    if ((g_force_lut_rx_flags & FORCE_LUT_RX_READY) != FORCE_LUT_RX_READY) {
        return 0U;
    }
    if (force_lut_part_data_valid(&g_force_lut_rx) == 0) {
        return 0U;
    }

    part_mask = force_lut_part_mask(g_force_lut_rx.part_id);
    if (part_mask == 0U) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (FLASH_OK == flash_init()) {
        if (force_lut_flash_write_part(&g_force_lut_rx) != 0) {
            write_ok = 1U;
        }
        (void)flash_deinit();
    }
    if (0U == primask) {
        __enable_irq();
    }

    if (write_ok == 0U) {
        return 0U;
    }

    g_force_lut_written_mask |= part_mask;
    loaded_count = force_lut_flash_load_all();

    if ((g_force_lut_written_mask == FORCE_LUT_ALL_PARTS_MASK) &&
        (loaded_count == FORCE_LUT_PART_COUNT)) {
        (void)pressure_mode_write_flag(1U);
    } else if (press_flag) {
        (void)pressure_mode_write_flag(0U);
    }

    return 1U;
}

static uint8_t force_lut_handle_packet(package_t *package, uint8_t cmd)
{
    uint8_t  lut_packet_id = (uint8_t)(cmd >> 4);
    uint8_t  cmd_type      = (uint8_t)(cmd & 0x0FU);
    uint8_t  count;
    uint16_t expected_count;

    if (cmd_type != 0x03U) {
        return 0U;
    }
    if (package_remain_size(package) < 1U) {
        return 1U;
    }

    count = package_read_u8(package);

    switch (lut_packet_id) {
    case 0U: {
        float meta[6];

        if ((count != 6U) || (force_lut_read_float_array(package, meta, count, 6U) == 0U)) {
            return 1U;
        }

        memset(&g_force_lut_rx, 0, sizeof(g_force_lut_rx));
        g_force_lut_rx.part_id           = (uint8_t)force_lut_float_to_u16(meta[0]);
        g_force_lut_rx.sensor_rows       = force_lut_float_to_u16(meta[1]);
        g_force_lut_rx.sensor_cols       = force_lut_float_to_u16(meta[2]);
        g_force_lut_rx.area_num          = force_lut_float_to_u16(meta[3]);
        g_force_lut_rx.force_num         = force_lut_float_to_u16(meta[4]);
        g_force_lut_rx.has_shape_feature = (uint8_t)force_lut_float_to_u16(meta[5]);

        if ((force_lut_part_index(g_force_lut_rx.part_id) < 0) ||
            (g_force_lut_rx.area_num == 0U) ||
            (g_force_lut_rx.area_num > FORCE_LUT_MAX_AREA_NUM) ||
            (g_force_lut_rx.force_num == 0U) ||
            (g_force_lut_rx.force_num > FORCE_LUT_MAX_FORCE_NUM) ||
            (((uint32_t)g_force_lut_rx.area_num * (uint32_t)g_force_lut_rx.force_num) >
             (uint32_t)FORCE_LUT_MAX_TABLE_COUNT)) {
            g_force_lut_rx_flags = 0U;
            return 1U;
        }

        g_force_lut_rx_flags = FORCE_LUT_RX_META;
        return 1U;
    }
    case 1U:
        if ((g_force_lut_rx_flags & FORCE_LUT_RX_META) == 0U) {
            return 1U;
        }
        if ((count == g_force_lut_rx.area_num) &&
            (force_lut_read_float_array(package, g_force_lut_rx.area_cx, count,
                                        FORCE_LUT_MAX_AREA_NUM) != 0U)) {
            g_force_lut_rx_flags |= FORCE_LUT_RX_AREA_CX;
        }
        return 1U;
    case 2U:
        if ((g_force_lut_rx_flags & FORCE_LUT_RX_META) == 0U) {
            return 1U;
        }
        if ((count == g_force_lut_rx.area_num) &&
            (force_lut_read_float_array(package, g_force_lut_rx.area_cy, count,
                                        FORCE_LUT_MAX_AREA_NUM) != 0U)) {
            g_force_lut_rx_flags |= FORCE_LUT_RX_AREA_CY;
        }
        return 1U;
    case 3U:
        if ((g_force_lut_rx_flags & FORCE_LUT_RX_META) == 0U) {
            return 1U;
        }
        if ((count == g_force_lut_rx.force_num) &&
            (force_lut_read_float_array(package, g_force_lut_rx.force_grid, count,
                                        FORCE_LUT_MAX_FORCE_NUM) != 0U)) {
            g_force_lut_rx_flags |= FORCE_LUT_RX_FORCE_GRID;
        }
        return 1U;
    case 4U:
        if ((g_force_lut_rx_flags & FORCE_LUT_RX_META) == 0U) {
            return 1U;
        }
        expected_count = (uint16_t)(g_force_lut_rx.area_num * g_force_lut_rx.force_num);
        if ((count == expected_count) &&
            (force_lut_read_float_array(package, g_force_lut_rx.sensor_table, count,
                                        FORCE_LUT_MAX_TABLE_COUNT) != 0U)) {
            g_force_lut_rx_flags |= FORCE_LUT_RX_SENSOR_TABLE;
            (void)force_lut_write_ready_part();
        }
        return 1U;
    default:
        return 1U;
    }
}

void config_execute_cmd(package_t *package)
{
    uint8_t addr = package->channel >> 4;
    switch (package->channel & 0x0f) {
    case ATTRIBUTE: {
        //if (addr != config_.address) return;
        uint8_t cmd = package_read_u8(package);
        switch (cmd) {
        case 0x01: {
            const char *version_str = APPLICATION_VERSION_FULL;
            uint8_t     flags       = (package->flags & 0xFC) | 0x02;

            serial_async_send(&serial1, /*(config_.address << 4) |*/ 0x01, flags, (uint8_t *)version_str,
                              strlen(version_str) + 1);
            break;
        }
        case 0x02:
		 serial_async_send(&serial1, /*(config_.address << 4) |*/ 0x01,
                  (package->flags & 0xFC) | 0x02, &press_flag, 1);
          break;
        case 0x03: break;
        case 0x04: break;
        case 0x05: {
            uint8_t uid[32] = { 0 };
            retrieve_chip_id(uid);
            serial_async_send(&serial1, /*(config_.address << 4) |*/ 0x01, (package->flags & 0xFC) | 0x02, uid,
                              32);
            break;
        }
        case 0x06:
            serial_async_send(&serial1, /*(config_.address << 4) |*/ 0x01, (package->flags & 0xFC) | 0x02,
                              &config_.address, 1);
            break;
        default: break;
        }
        break;
    }
    case DATA: {
        uint8_t flags;
        uint8_t *upload_buffer;
        uint16_t upload_size;

        if (addr != config_.address) break;
        (void)package_read_u8(package);
        flags = package->flags & 0xFC;
        package_id = flags;
        upload_buffer = system_get_upload_buffer();
        upload_size = system_get_upload_size();

        if ((config_.zeroing.finished) && (upload_buffer != NULL) && (upload_size != 0U) && (serial1.txmtx == 0U)) {
            if (config_.multiple_serial_enabled) {
                serial_async_send(&serial1,
                                  (config_.address << 4) | 0x02,
                                  flags | 0x03,
                                  upload_buffer,
                                  upload_size);
            } else {
                serial_async_send(&serial1, 0x02, 0X00, upload_buffer, upload_size);
            }

            if (serial1.txmtx != 0U) {
                package_id = 0xff;
            }
        }
        break;
    }
    case 0x06: {
        uint8_t cmd = package_read_u8(package);
        uint8_t value = package_read_u8(package);

        if (cmd == 0x01U) {
            system_set_upload_paused(value == 0x00U);
        }
        break;
    }

    case ZERO: {
        uint8_t cmd = package_read_u8(package);
        if (force_lut_handle_packet(package, cmd) != 0U) {
            break;
        }
        switch (cmd) {
        case CONFIG_CALIBRATION: {
            uint8_t value = package_read_u8(package);
            switch (value) {
            case 0x00: calibration_enable(&config_, 0x00); break;
            case 0x01: calibration_enable(&config_, 0x01); break;
            case 0x02:
                calibration_clear(&config_);
                system_rect_zero_clear();
                break;
            default:   break;
            }
            break;
        }
            // calibration parameters
        case CONFIG_ZEROING: {
            config_.zeroing.method       = package_read_u8(package);
            config_.zeroing.frames       = package_read_u16(package);
            config_.zeroing.alpha        = package_read_f32(package);
            config_.zeroing.static_beta  = package_read_u16(package);
            config_.zeroing.dynamic_beta = package_read_f32(package);
            config_.zeroing.type         = package_read_u8(package);
            calibration_clear(&config_);
            system_rect_zero_clear();
            break;
        }
        case CONFIG_CALIBRATION_COEFFICIENT_1: {
            uint8_t au8TableBuf[PRESS_LUT_MAX_SIZE * 2U * sizeof(float)] = { 0U };
            uint8_t u8Count = package->payload[1];
            uint32_t u32Primask;
            uint16_t u16Bytes;
            uint16_t u16CopyBytes;

            if (u8Count > PRESS_LUT_MAX_SIZE) {
                u8Count = PRESS_LUT_MAX_SIZE;
            }

            u16Bytes = (uint16_t)(u8Count * 2U * sizeof(float));
            u16CopyBytes = 0U;
            if (package->length > 2U) {
                u16CopyBytes = (uint16_t)(package->length - 2U);
                if (u16CopyBytes > u16Bytes) {
                    u16CopyBytes = u16Bytes;
                }
            }
            if (u16Bytes > 0U) {
                (void)memcpy(au8TableBuf, &package->payload[2], u16CopyBytes);
            }

            press_flag = 1U;
            u32Primask = __get_PRIMASK();
            __disable_irq();
            if (FLASH_OK == flash_init()) {
                (void)flash_pack_and_write(ADDR_FLASH_SECTOR_PRESS_LUT_DATA, au8TableBuf, u16Bytes);
                (void)flash_pack_and_write(ADDR_FLASH_SECTOR_PRESS_MODE_FLAG, (uint8_t *)&press_flag, 1U);
                (void)flash_pack_and_write(ADDR_FLASH_SECTOR_PRESS_LUT_COUNT, &u8Count, 1U);
                (void)flash_deinit();
            }
            if (0U == u32Primask) {
                __enable_irq();
            }
            NVIC_SystemReset();
            break;
        }
#if (PRESS_CALIBRATION_AREA_COUNT > 1U)
		case CONFIG_CALIBRATION_COEFFICIENT_2: {
            uint8_t num_buf[514] = { 0 };
            for(int i=0;i<package->payload[1]*2*4;i++)
			{
				num_buf[i] =  package->payload[i+2];
			}
            num_buf[512] = 0x01;                                          // 鍘嬪姏妯″紡鏍囧織浣?            num_buf[513] =  package->payload[1];                          //姊害鏌ヨ〃鏁伴噺
			press_lut_flag[1]=1;
            flash_init();
            flash_pack_and_write(ADDR_FLASH_SECTOR_33, num_buf, num_buf[513]*2*4);     // 鍐欑郴鏁?			if(press_lut_flag[0]==1&&press_lut_flag[1]==1&&press_lut_flag[2]==1&&press_lut_flag[3]==1)
				flash_pack_and_write(ADDR_FLASH_SECTOR_PRESS_MODE_FLAG, num_buf + 512, 1); // 鍐欏帇鍔涙爣蹇椾綅
			flash_pack_and_write(ADDR_FLASH_SECTOR_36, num_buf + 513, 1);   // 鍐欐搴︽煡琛ㄦ暟閲?            flash_deinit();
            if(press_lut_flag[0]==1&&press_lut_flag[1]==1&&press_lut_flag[2]==1&&press_lut_flag[3]==1)
			{
				__disable_irq();
                NVIC_SystemReset();
			}
            //press_flag = true;
            // iap_write_appbin(ADDR_FLASH_SECTOR_30,num_buf,12,0);
            break;
        }
		case CONFIG_CALIBRATION_COEFFICIENT_3: {
            uint8_t num_buf[514] = { 0 };
            for(int i=0;i<package->payload[1]*2*4;i++)
			{
				num_buf[i] =  package->payload[i+2];
			}
            num_buf[512] = 0x01;                                          // 鍘嬪姏妯″紡鏍囧織浣?            num_buf[513] =  package->payload[1];                          //姊害鏌ヨ〃鏁伴噺
			press_lut_flag[2]=1;
            flash_init();
            flash_pack_and_write(ADDR_FLASH_SECTOR_34, num_buf, num_buf[513]*2*4);     // 鍐欑郴鏁?			if(press_lut_flag[0]==1&&press_lut_flag[1]==1&&press_lut_flag[2]==1&&press_lut_flag[3]==1)
				flash_pack_and_write(ADDR_FLASH_SECTOR_PRESS_MODE_FLAG, num_buf + 512, 1); // 鍐欏帇鍔涙爣蹇椾綅
			flash_pack_and_write(ADDR_FLASH_SECTOR_37, num_buf + 513, 1);   // 鍐欐搴︽煡琛ㄦ暟閲?            flash_deinit();
			if(press_lut_flag[0]==1&&press_lut_flag[1]==1&&press_lut_flag[2]==1&&press_lut_flag[3]==1)
			{
				__disable_irq();
                NVIC_SystemReset();
			}
           // press_flag = true;
            // iap_write_appbin(ADDR_FLASH_SECTOR_30,num_buf,12,0);
            break;
        }
		case CONFIG_CALIBRATION_COEFFICIENT_4: {
            uint8_t num_buf[514] = { 0 };
            for(int i=0;i<package->payload[1]*2*4;i++)
			{
				num_buf[i] =  package->payload[i+2];
			}
            num_buf[512] = 0x01;                                          // 鍘嬪姏妯″紡鏍囧織浣?            num_buf[513] =  package->payload[1];                          //姊害鏌ヨ〃鏁伴噺
			press_lut_flag[3]=1;
            flash_init();
            flash_pack_and_write(ADDR_FLASH_SECTOR_35, num_buf, num_buf[513]*2*4);     // 鍐欑郴鏁?			if(press_lut_flag[0]==1&&press_lut_flag[1]==1&&press_lut_flag[2]==1&&press_lut_flag[3]==1)
				flash_pack_and_write(ADDR_FLASH_SECTOR_PRESS_MODE_FLAG, num_buf + 512, 1); // 鍐欏帇鍔涙爣蹇椾綅
			flash_pack_and_write(ADDR_FLASH_SECTOR_38, num_buf + 513, 1);   // 鍐欐搴︽煡琛ㄦ暟閲?            flash_deinit();
			if(press_lut_flag[0]==1&&press_lut_flag[1]==1&&press_lut_flag[2]==1&&press_lut_flag[3]==1)
			{
				__disable_irq();
                NVIC_SystemReset();
			}
            //press_flag = true;
            // iap_write_appbin(ADDR_FLASH_SECTOR_30,num_buf,12,0);
            break;
        }
#endif
        default: break;
        }
        break;
    }
    case CONFIG: {
		 __disable_irq();
        uint8_t cmd = package_read_u8(package);
        switch (cmd) {
        case CONFIG_MULTIPLE_SERIAL: {
            config_.multiple_serial_enabled = package_read_u8(package);
            break;
        }
        case CONFIG_FILTER: {
            config_.filter_enabled = package_read_u8(package);
            break;
        }
        case CONFIG_BYTES_NUM: {
            config_.bytes_num = package_read_u8(package);
            break;
        }
        case CONFIG_ADDRESS: {
            uint8_t id = package_read_u8(package);
           if (id != 0x0) {
               config_.address = id;
           }
            break;
        }
        default: break;
        }
        flash_init();
        config_write(&config_, ADDR_FLASH_SECTOR_CONFIG);
        flash_deinit();
         __enable_irq();
       // NVIC_SystemReset();
        break;
    }
    case OTA: {
        request_ota_upgrade();
        system_prepare_for_reset();
        NVIC_SystemReset();
        break;
    }
		case 0x0F: // 鍘嬪姏妯″紡鏍囧織浣嶄慨鏀瑰懡浠?  3C 3C FF 00 01 00 00 00 00 3E 3E  鍏抽棴鍘嬪姏   3C 3C FF 00 01 00 01 C1 C0 3E 3E  鎵撳紑鍘嬪姏
    {
        uint8_t temp_flag = package->payload[0];

        if (temp_flag == 0U) {
            g_force_lut_written_mask = 0U;
            force_lut_clear_all_parts();
            (void)pressure_mode_write_flag(0U);
        } else if (force_lut_flash_load_all() == FORCE_LUT_PART_COUNT) {
            g_force_lut_written_mask = FORCE_LUT_ALL_PARTS_MASK;
            (void)pressure_mode_write_flag(1U);
        } else {
            (void)pressure_mode_write_flag(0U);
        }

        // 鐩存帴鍒锋柊鍏ㄥ眬鍙橀噺锛岀‘淇濆拰flash涓€鑷?        extern void get_pressure_cal_cfg();
        get_pressure_cal_cfg();
        break;
    }
    default: break;
    }
}

void retrieve_chip_id(uint8_t *uid)
{
    stc_efm_unique_id_t stcUid = { 0 };

    EFM_GetUID(&stcUid);
    snprintf((char *)uid, 27, "%08lX-%08lX-%08lX",
             (unsigned long)stcUid.u32UniqueID0,
             (unsigned long)stcUid.u32UniqueID1,
             (unsigned long)stcUid.u32UniqueID2);
}

static uint32_t calculate_config_crc(const config_t *config)
{
    return crc_16((const uint8_t *)config, sizeof(config_t));
}

void config_init_default(config_t *config)
{
    config->load_cfg                = false;
    config->filter_enabled          = false;
    config->multiple_serial_enabled = true;
    config->bytes_num               = DOUBLE_BYTE;
    config->address  = 0x06;
    config->iap_flag = false;
    config->app_flag = true;

    config->zeroing = (zeroing_t){ .enabled      = true,
                                   .method       = ADAPTIVE_MAX,
                                   .frames       = 64,
                                   .alpha        = 1.1f,
                                   .static_beta  = 20,
                                   .dynamic_beta = 0.0f,
                                   .type         = THRESHOLD_TOZERO,
                                   .finished     = false };
    config->calibration =
        (calibration_t){ .coefficient_a = 0, .coefficient_b = 0, .coefficient_c = 0, .force = 0 };

    memset(config->reserved, 0, sizeof(config->reserved));
}

static void pack_config(const config_t *config, uint8_t *buffer)
{
    config_header_t *header = (config_header_t *)buffer;

    header->magic = CONFIG_MAGIC;
    header->size  = sizeof(config_t);
    header->crc   = calculate_config_crc(config);

    memcpy(buffer + sizeof(config_header_t), config, sizeof(config_t));
}

bool config_write(const config_t *config, uint32_t flash_addr)
{
    enum {
        CONFIG_PACKED_SIZE = ((sizeof(config_header_t) + sizeof(config_t)) + 7U) / 8U * 8U
    };

    if (flash_addr % 8 != 0) {
        // logi("\r\n Error: Flash address not aligned to 8 bytes");
        return false;
    }

    uint64_t write_buffer_u64[CONFIG_PACKED_SIZE / 8U] = { 0 };
    uint8_t *write_buffer = (uint8_t *)write_buffer_u64;
    pack_config(config, write_buffer);

    uint8_t rst = flash_pack_and_write(flash_addr, write_buffer, CONFIG_PACKED_SIZE);
    if (rst != FLASH_OK) {
        // logi("\r\nError: Flash write failed  ");
        return false;
    }
    return true;
}

flash_unpack_t unpack_config(const uint8_t *buffer, config_t *config)
{
    const config_header_t *header = (const config_header_t *)buffer;

    if (header->magic != CONFIG_MAGIC) {
        return MAGIC_ERROR;
    }
    if (header->size != sizeof(config_t)) {
        return SIZE_ERROR;
    }
    memcpy(config, buffer + sizeof(config_header_t), sizeof(config_t));
    if (header->crc != calculate_config_crc(config)) {
        return CRC_ERROR;
    }
    return UNPACK_OK;
}

bool config_read(config_t *config, uint32_t flash_addr)
{
    enum {
        CONFIG_PACKED_SIZE = ((sizeof(config_header_t) + sizeof(config_t)) + 7U) / 8U * 8U
    };

    if (flash_addr % 8 != 0) {
        // logi("\r\n Error: Flash address not aligned to 8 bytes\n");
        return false;
    }

    uint64_t read_buffer_u64[CONFIG_PACKED_SIZE / 8U];
    uint8_t *read_buffer = (uint8_t *)read_buffer_u64;
    memset(read_buffer, 0, CONFIG_PACKED_SIZE);
    if (flash_read(flash_addr, read_buffer, CONFIG_PACKED_SIZE) != FLASH_OK) {
        // logi("\r\n Error: Flash read failed\n");
        return false;
    }
    uint8_t rst = unpack_config(read_buffer, config);
    if (rst != UNPACK_OK) {
        // logi("\r\n Error: Unpack config failed (CRC or magic mismatch)");
        return false;
    }

    return true;
}

