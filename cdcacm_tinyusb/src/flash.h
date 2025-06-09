
#ifndef _FLASH_H_
#define _FLASH_H_


// defines for STM32G0B1
#define FLASH_PAGE_SIZE   2048
#define FLASH_ROW_SIZE     256
#define ROW_DWORDS       (FLASH_ROW_SIZE / 8)
#define NUM_ROWS         (FLASH_PAGE_SIZE / FLASH_ROW_SIZE)

#define MAGIC_KEY1 0x45670123
#define MAGIC_KEY2 0xCDEF89AB


typedef enum {
    FLASH_OK = 0,
    FLASH_ERR_ALIGNMENT,
    FLASH_ERR_BUSY,
    FLASH_ERR_PROGRAM,
    FLASH_ERR_ERASE,
    FLASH_ERR_UNKNOWN
} FlashStatus;

void ResetMCU(void);
FlashStatus Flash_ProgramFullPage(uint32_t current_page, void *source_data);

#endif // _FLASH_H_