
#include <string.h>
#include "stm32g0xx.h"
#include "flash.h"


#ifndef FLASH_SR_BSY
#define FLASH_SR_BSY (FLASH_SR_BSY1 | FLASH_SR_BSY2)
#endif

void ResetMCU(void){
    __NVIC_SystemReset();
}


void JumpToApplication(uint32_t user_code_offset){

    typedef void (*pFunction)(void);
    pFunction Jump_To_Application;
    uint32_t JumpAddress;
    
        
        __disable_irq();
    NVIC->ICER[0] = 0xFFFFFFFF; // Disable interrupts
    NVIC->ICPR[0] = 0xFFFFFFFF; // Clear pending
    SysTick->CTRL = 0;          // Stop SysTick
    RCC->CSR |= RCC_CSR_RMVF;    
    
    JumpAddress = *(__IO uint32_t*) (FLASH_BASE + user_code_offset + 4);
    Jump_To_Application = (pFunction) JumpAddress;
    SCB->VTOR = FLASH_BASE + user_code_offset;
    __set_MSP(*(uint32_t *) (FLASH_BASE + user_code_offset));
    Jump_To_Application(); 
    while(1){}; // we should never get here
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

__attribute__((section(".ramfunc"))) FlashStatus Flash_ErasePage(uint32_t current_page) {
    while (FLASH->SR & FLASH_SR_BSY){};

    FLASH->CR &= ~FLASH_CR_PNB_Msk;
    FLASH->CR |= FLASH_CR_PER | (current_page << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;

    while (FLASH->SR & FLASH_SR_CFGBSY){};

    FLASH->CR &= ~FLASH_CR_PER;

    // Optional: error checking
    if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PROGERR)) {
        return FLASH_ERR_ERASE;
    }

    return FLASH_OK;
}


__attribute__((section(".ramfunc"))) FlashStatus Flash_ProgramDword(uint32_t dst_address, void *src_buffer) {
    // Check alignment
    // if ((uint32_t)src_buffer % 8 != 0) {
        // return FLASH_ERR_ALIGNMENT_SRC;  // Must be 64-bit aligned
    // }
    
    // if (dst_address % 8 != 0) {
        // return FLASH_ERR_ALIGNMENT_DST;  // Must be 64-bit aligned
    // }
    
    // 1. Wait if busy
    while (FLASH->SR & FLASH_SR_BSY) {};

    // 2. clear all previous programming error flags
    FLASH->SR = (FLASH_SR_OPTVERR | 
                 FLASH_SR_RDERR | 
                 FLASH_SR_FASTERR | 
                 FLASH_SR_MISERR | 
                 FLASH_SR_PGSERR | 
                 FLASH_SR_SIZERR | 
                 FLASH_SR_PGAERR | 
                 FLASH_SR_WRPERR | 
                 FLASH_SR_PROGERR | 
                 FLASH_SR_OPERR);

    // 3. check that CFGBSY bit of FLASH_SR is cleared
    while (FLASH->SR & FLASH_SR_CFGBSY) {};
    
    // 4. set the PG bit of the FLASH_SR
    FLASH->CR |= FLASH_CR_PG;
    
    // 5. perform data write
    // This is the **key step**: write 256 bytes at once
    //memcpy((void *)dst_address, src_buffer, FLASH_ROW_SIZE);
    *(uint64_t*)dst_address = *(uint64_t*)src_buffer;
    
    // 6. Wait until CFGSY bit FLASH_SR is cleared
    while (FLASH->SR & (FLASH_SR_CFGBSY | FLASH_SR_BSY)) {};

    // 7. Check that EOP flag of FLASH_SR is set (=programming success)
    // and clear it by software (only when EOPIE is enabled)
    // if (FLASH->SR & FLASH_SR_EOP){
        // FLASH->SR |= FLASH_SR_EOP;
    // }
    // else {
        // return FLASH_ERR_EOP;
    // }
    
    // 8. Clear the PG bit of FLASH_CR if no more programming is needed
    FLASH->CR &= ~FLASH_CR_PG;

    // // Check for errors
    // if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR)) {
        // return FLASH_ERR_PROGRAM;
    // }

    return FLASH_OK;
}




// //__attribute__((section(".ramfunc")))  // Optional but safer
// FlashStatus Flash_FastProgramRow(uint32_t dst_address, const void *src_buffer) {
    // // Check alignment
    // if ((uint32_t)src_buffer % 8 != 0) {
        // return FLASH_ERR_ALIGNMENT_SRC;  // Must be 64-bit aligned
    // }
    
    // if (dst_address % 8 != 0) {
        // return FLASH_ERR_ALIGNMENT_DST;  // Must be 64-bit aligned
    // }
    
    // // Wait if busy
    // while (FLASH->SR & FLASH_SR_BSY);

    // // Enable fast programming
    // FLASH->CR |= FLASH_CR_FSTPG;

    // // This is the **key step**: write 256 bytes at once
    // memcpy((void *)dst_address, src_buffer, FLASH_ROW_SIZE);

    // // Wait for programming to complete
    // while (FLASH->SR & FLASH_SR_BSY);

    // // Clear fast programming bit
    // FLASH->CR &= ~FLASH_CR_FSTPG;

    // // Check for errors
    // if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR)) {
        // return FLASH_ERR_PROGRAM;
    // }

    // return FLASH_OK;
// }

FlashStatus Flash_ProgramFullPage(uint32_t current_page, void *source_data) {
    FlashStatus status;

    Flash_Unlock();
    
    status = Flash_ErasePage(current_page);
    
    if (status) {
        Flash_Lock();
        return status;
    }


    uint32_t page_address = FLASH_BASE + current_page * FLASH_PAGE_SIZE;

    // Program each row
    uint8_t* src_ptr = (uint8_t *)source_data;
    uint32_t dst_addr = page_address;

    //status = Flash_ProgramDword(dst_addr, src_ptr);

    for (int i = 0; i < FLASH_PAGE_SIZE; i += FLASH_DWORD_SIZE) {

        status = Flash_ProgramDword(dst_addr, src_ptr);
        if (status) {
            Flash_Lock();
            return status;
        }
        src_ptr += FLASH_DWORD_SIZE;
        dst_addr += FLASH_DWORD_SIZE;
    }

    
      
    if (status) {
        Flash_Lock();
        return status;
    }


    //status = Flash_ProgramDword(dst_addr, src_ptr);
    

    // for (int i = 0; i < NUM_ROWS; ++i) {
        // dst_addr += FLASH_ROW_SIZE;
        // src_ptr += FLASH_ROW_SIZE;
        // status = Flash_FastProgramRow(dst_addr, src_ptr);
            
        // if (status) {
            // Flash_Lock();
            // return status;
        // }
    // }

    // Lock flash
    Flash_Lock();
    return FLASH_OK;
}