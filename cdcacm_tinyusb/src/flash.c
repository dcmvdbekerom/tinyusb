
#include <string.h>
#include "stm32g0xx.h"
#include "flash.h"


#ifndef FLASH_SR_BSY
#define FLASH_SR_BSY (FLASH_SR_BSY1 || FLASH_SR_BSY2)
#endif

void ResetMCU(void){
    __NVIC_SystemReset();
}

uint32_t Flash_Calc_Address(uint32_t offset){
    return FLASH_BASE + offset;
}

void Flash_Unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = MAGIC_KEY1;
        FLASH->KEYR = MAGIC_KEY2;
    }
}

void Flash_Lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

FlashStatus Flash_ErasePage(uint32_t page_address) {
    while (FLASH->SR & FLASH_SR_BSY);

    uint32_t page_num = (page_address - FLASH_BASE) / FLASH_PAGE_SIZE;

    FLASH->CR &= ~FLASH_CR_PG_Msk;
    FLASH->CR |= FLASH_CR_PER | (page_num << FLASH_CR_PG_Pos);
    FLASH->CR |= FLASH_CR_STRT;

    while (FLASH->SR & FLASH_SR_BSY);

    FLASH->CR &= ~FLASH_CR_PER;

    // Optional: error checking
    if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PROGERR)) {
        return FLASH_ERR_ERASE;
    }

    return FLASH_OK;
}

__attribute__((section(".ramfunc")))  // Optional but safer
FlashStatus Flash_FastProgramRow(uint32_t dst_address, const void *src_buffer) {
    // Check alignment
    if (((uint32_t)src_buffer % 8 != 0) || (dst_address % 8 != 0)) {
        return FLASH_ERR_ALIGNMENT;  // Must be 64-bit aligned
    }

    // Wait if busy
    while (FLASH->SR & FLASH_SR_BSY);

    // Enable fast programming
    FLASH->CR |= FLASH_CR_FSTPG;

    // This is the **key step**: write 256 bytes at once
    memcpy((void *)dst_address, src_buffer, FLASH_ROW_SIZE);

    // Wait for programming to complete
    while (FLASH->SR & FLASH_SR_BSY);

    // Clear fast programming bit
    FLASH->CR &= ~FLASH_CR_FSTPG;

    // Check for errors
    if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR)) {
        return FLASH_ERR_PROGRAM;
    }

    return FLASH_OK;
}


__attribute__((aligned(8))) uint8_t flash_page_buffer[FLASH_PAGE_SIZE];

FlashStatus Flash_ProgramFullPage(uint32_t current_page, void *source_data) {
    FlashStatus status;

    Flash_Unlock();
    uint32_t page_address = FLASH_BASE + current_page * FLASH_PAGE_SIZE;
    status = Flash_ErasePage(page_address);
    if (status) {
        Flash_Lock();
        return status;
    }

    // Program each row
    uint8_t* src_ptr = (uint8_t *)source_data;
    uint32_t dst_addr = page_address;
        
    for (int i = 0; i < NUM_ROWS; ++i) {
        dst_addr += FLASH_ROW_SIZE;
        src_ptr += FLASH_ROW_SIZE;
        status = Flash_FastProgramRow(dst_addr, src_ptr);
            
        if (status) {
            Flash_Lock();
            return status;
        }
    }

    // Lock flash
    Flash_Lock();
    return FLASH_OK;
}