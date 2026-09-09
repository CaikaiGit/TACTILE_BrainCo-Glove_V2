#ifndef __APP_BASE_H__
#define __APP_BASE_H__

#include "hc32_ll.h"
#include "hc32_ll_efm.h"

#ifndef APP_CODE_BASE_VALUE
#define APP_CODE_BASE_VALUE         (0x00008000UL)
#endif

#define APP_CODE_BASE               (APP_CODE_BASE_VALUE)
#define APP_CODE_SIZE               ((EFM_END_ADDR + 1UL) - APP_CODE_BASE)

#endif
