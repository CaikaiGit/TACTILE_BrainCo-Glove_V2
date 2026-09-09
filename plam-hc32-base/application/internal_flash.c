#include "internal_flash.h"

#include <string.h>

static flash_state_t volatile flash_current_state = FLASH_UNINITIALIZED;

static uint8_t flash_addr_valid(uint32_t addr, uint32_t size)
{
    if (size == 0UL) {
        return 0U;
    }
    if (addr < APP_FLASH_BASE) {
        return 0U;
    }
    if (addr >= APP_FLASH_END) {
        return 0U;
    }
    if (size > (APP_FLASH_END - addr)) {
        return 0U;
    }
    return 1U;
}

static flash_status_t flash_erase_range(uint32_t addr, uint32_t size)
{
    uint32_t sector_addr;
    uint32_t sector_end;

    if (!flash_addr_valid(addr, size)) {
        return FLASH_ERROR_ADDR;
    }

    sector_addr = addr & ~(FLASH_PAGE_SIZE - 1UL);
    sector_end = (addr + size - 1UL) & ~(FLASH_PAGE_SIZE - 1UL);

    while (sector_addr <= sector_end) {
        if (LL_OK != EFM_SectorErase(sector_addr)) {
            return FLASH_ERROR_ERASE;
        }
        sector_addr += FLASH_PAGE_SIZE;
    }

    return FLASH_OK;
}

static flash_status_t flash_program_range(uint32_t addr, const uint8_t *data, uint32_t size)
{
    if (!flash_addr_valid(addr, size) || (data == NULL)) {
        return FLASH_ERROR_ADDR;
    }
    if ((addr % 4UL) != 0UL) {
        return FLASH_ERROR_ADDR;
    }

    if (LL_OK != EFM_Program(addr, data, size)) {
        return FLASH_ERROR_WRITE;
    }

    return FLASH_OK;
}

flash_status_t flash_init(void)
{
    while (SET != EFM_GetStatus(EFM_FLAG_RDY)) {
    }
    EFM_REG_Unlock();
    EFM_FWMC_Cmd(ENABLE);
    flash_current_state = FLASH_READY;
    return FLASH_OK;
}

flash_status_t flash_deinit(void)
{
    EFM_FWMC_Cmd(DISABLE);
    flash_current_state = FLASH_UNINITIALIZED;
    return FLASH_OK;
}

flash_state_t flash_get_state(void)
{
    return flash_current_state;
}

flash_status_t flash_read_word(uint32_t addr, uint32_t *data)
{
    if ((data == NULL) || !flash_addr_valid(addr, sizeof(uint32_t))) {
        return FLASH_ERROR_ADDR;
    }
    if ((addr % 4UL) != 0UL) {
        return FLASH_ERROR_ADDR;
    }

    *data = *(__IO uint32_t *)addr;
    return FLASH_OK;
}

flash_status_t flash_read(uint32_t addr, void *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return FLASH_ERROR_SIZE;
    }
    if (LL_OK != EFM_ReadByte(addr, (uint8_t *)data, length)) {
        return FLASH_ERROR_READ;
    }
    return FLASH_OK;
}

flash_status_t flash_erase_sector(uint32_t addr)
{
    flash_status_t status;

    if (flash_current_state != FLASH_READY) {
        return FLASH_ERROR_BUSY;
    }
    if ((addr % 4UL) != 0UL) {
        return FLASH_ERROR_ADDR;
    }

    flash_current_state = FLASH_BUSY;
    status = flash_erase_range(addr, FLASH_PAGE_SIZE);
    flash_current_state = FLASH_READY;
    return status;
}

flash_status_t flash_write(uint32_t addr, const uint64_t *data, uint16_t length)
{
    uint32_t size;
    flash_status_t status;

    if (flash_current_state != FLASH_READY) {
        return FLASH_ERROR_BUSY;
    }
    if ((data == NULL) || (length == 0U)) {
        return FLASH_ERROR_SIZE;
    }
    if ((addr % 4UL) != 0UL) {
        return FLASH_ERROR_ADDR;
    }

    size = (uint32_t)length * sizeof(uint64_t);
    if (!flash_addr_valid(addr, size)) {
        return FLASH_ERROR_ADDR;
    }

    flash_current_state = FLASH_BUSY;

    status = flash_erase_range(addr, size);
    if (status != FLASH_OK) {
        flash_current_state = FLASH_READY;
        return status;
    }

    status = flash_program_range(addr, (const uint8_t *)data, size);
    if (status != FLASH_OK) {
        flash_current_state = FLASH_READY;
        return status;
    }

    flash_current_state = FLASH_READY;
    return FLASH_OK;
}

flash_status_t flash_verify(uint32_t addr, const uint64_t *data, uint16_t length)
{
    uint32_t size;

    if ((data == NULL) || (length == 0U)) {
        return FLASH_ERROR_SIZE;
    }

    size = (uint32_t)length * sizeof(uint64_t);
    if (!flash_addr_valid(addr, size)) {
        return FLASH_ERROR_ADDR;
    }

    if (memcmp((const void *)addr, data, size) != 0) {
        return FLASH_ERROR_VERIFY;
    }

    return FLASH_OK;
}

flash_status_t flash_write_with_verify(uint32_t addr, const uint64_t *data, uint16_t length)
{
    flash_status_t status = flash_write(addr, data, length);
    if (status != FLASH_OK) {
        return status;
    }

    return flash_verify(addr, data, length);
}

flash_status_t flash_pack_and_write(uint32_t addr, const uint8_t *data, uint32_t size)
{
    static uint8_t verify_buf[FLASH_PACK_WRITE_MAX_SIZE];

    if (flash_current_state != FLASH_READY) {
        return FLASH_ERROR_BUSY;
    }
    if ((data == NULL) || (size == 0UL) || (size > sizeof(verify_buf))) {
        return FLASH_ERROR_SIZE;
    }
    flash_current_state = FLASH_BUSY;
    if (LL_OK != EFM_SectorErase(addr)) {
        flash_current_state = FLASH_READY;
        return FLASH_ERROR_ERASE;
    }
    if (LL_OK != EFM_SequenceProgram(addr, data, size)) {
        flash_current_state = FLASH_READY;
        return FLASH_ERROR_WRITE;
    }
    if (LL_OK != EFM_ReadByte(addr, verify_buf, size)) {
        flash_current_state = FLASH_READY;
        return FLASH_ERROR_READ;
    }
    if (0 != memcmp(verify_buf, data, size)) {
        flash_current_state = FLASH_READY;
        return FLASH_ERROR_VERIFY;
    }

    flash_current_state = FLASH_READY;
    return FLASH_OK;
}

uint8_t stmflash_get_flash_sector(uint32_t addr)
{
    if (!flash_addr_valid(addr, 1UL)) {
        return 0xFFU;
    }
    return (uint8_t)((addr - APP_FLASH_BASE) / FLASH_PAGE_SIZE);
}
