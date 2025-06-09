
#ifndef _FLASH_H_
#define _FLASH_H_


// defines for STM32G0B1
#define FLASH_PAGE_SIZE   2048
#define FLASH_ROW_SIZE     256
#define FLASH_DWORD_SIZE     8
#define ROW_DWORDS       (FLASH_ROW_SIZE / FLASH_DWORD_SIZE)
#define NUM_ROWS         (FLASH_PAGE_SIZE / FLASH_ROW_SIZE)

#define MAGIC_KEY1 0x45670123
#define MAGIC_KEY2 0xCDEF89AB


typedef enum {
    FLASH_OK = 0,
    FLASH_ERR_ALIGNMENT_SRC = 101,
    FLASH_ERR_ALIGNMENT_DST = 102,
    FLASH_ERR_BUSY = 103,
    FLASH_ERR_PROGRAM = 104,
    FLASH_ERR_ERASE = 105,
    FLASH_ERR_BUSY_PRG = 106,
    FLASH_ERR_EOP = 107,
    FLASH_ERR_UNKNOWN = 255
} FlashStatus;

void ResetMCU(void);
void JumpToApplication(uint32_t user_code_offset);
FlashStatus Flash_ProgramFullPage(uint32_t current_page, void *source_data);

#endif // _FLASH_H_