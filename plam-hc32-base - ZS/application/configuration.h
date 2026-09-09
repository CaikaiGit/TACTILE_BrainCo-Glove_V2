#ifndef APPLICATION_CONFIGURATION
#define APPLICATION_CONFIGURATION

#include "serial.h"
#include "version.h"

#include <stdbool.h>
void config_execute_cmd(package_t *package);
void read_address(void);
/* ���ñ�ʶ�� */
#define CONFIG_MAGIC 0xCF

#ifndef PRESS_CALIBRATION_AREA_COUNT
#define PRESS_CALIBRATION_AREA_COUNT 1U
#endif

typedef enum
{
    UNPACK_OK = 0,
    MAGIC_ERROR,
    SIZE_ERROR,
    CRC_ERROR
} flash_unpack_t;
/* ����ͷ�� */
typedef struct
{
    uint16_t magic; /* ��ʶ�� */
    uint16_t size;  /* ���ô�С */
    uint16_t crc;   /* CRCУ�� */
} config_header_t;

/*�궨����*/
typedef struct
{
    float coefficient_a;
    float coefficient_b;
    float coefficient_c;
    float force;
} calibration_t;

/* ������� */
typedef struct
{
    bool volatile enabled;
    uint8_t  method;
    uint16_t frames;
    float    alpha;
    uint16_t static_beta;
    float    dynamic_beta;
    uint8_t  type;
    bool volatile finished;
} zeroing_t;
enum
{
    SINGLE_BYTE,
    DOUBLE_BYTE
};

typedef struct
{
    zeroing_t     zeroing;
    calibration_t calibration;
    bool volatile load_cfg;
    bool volatile multiple_serial_enabled;
    bool volatile filter_enabled;
    bool volatile iap_flag;
    bool volatile app_flag;
	uint8_t bytes_num;
    uint8_t address;

    uint8_t reserved[64 - (7 + sizeof(zeroing_t) + sizeof(calibration_t))];
} config_t;

void config_execute_cmd(package_t *package);
void config_init_default(config_t *config);
bool config_write(const config_t *config, uint32_t flash_addr);
bool config_read(config_t *config, uint32_t flash_addr);

#endif // APPLICATION_CONFIGURATION


