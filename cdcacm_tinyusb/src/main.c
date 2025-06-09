/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "flash.h" 


#define USER_CODE_OFFSET  0x4000 //<USER CODE> flash start address (=16k).
#define CDCACM_BUF_SIZE 64


/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

static void led_blinking_task(void);
static void cdc_task(void);

static uint8_t CDC_MSG0[] = "<< debug window >>\r\n";
static uint8_t  CMD_SIGNATURE[7] = "BTLDCMD";

__attribute__((aligned(FLASH_DWORD_SIZE))) static uint8_t pageData[FLASH_PAGE_SIZE];
static volatile uint32_t current_Page = 128; //beyond last page to prevent writes
static volatile uint16_t currentPageOffset = 0;
//uint16_t erase_page = 1;    


void reset_pages(void);
void send_cmd(const char* cmd);
int write_page(uint32_t currentPage);
int write_last_page(uint32_t currentPage, uint32_t res);


/*------------- MAIN -------------*/
int main(void) {
  board_init();

  if (board_button_read()){
    JumpToApplication(USER_CODE_OFFSET);
  }


  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);
  
  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  while (1) {
      
      
    // if (board_button_read()){
        // blink_interval_ms = 1000;
        
    // }
    // else{
        // blink_interval_ms = 100;
    // }

      
  tud_task(); // tinyusb device task
  cdc_task();
  led_blinking_task();
    
  }
}


// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
  tud_cdc_n_write(0, CDC_MSG0, sizeof(CDC_MSG0) - 1);
  tud_cdc_n_write_flush(0);
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}


void Delay_ms(uint32_t ms) {
    uint32_t start = board_millis();
    while ((board_millis() - start) < ms);
}


void reset_pages(void){
    current_Page = USER_CODE_OFFSET / FLASH_PAGE_SIZE;
    currentPageOffset = 0;
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
static void cdc_task(void) {

    if (!tud_cdc_n_available(0)) return;

    uint8_t buf[CDCACM_BUF_SIZE];
    int count = tud_cdc_n_read(0, buf, sizeof(buf));
    
    if (count == 0) return;
    
    if (strncmp((char*) buf, (char*) CMD_SIGNATURE, sizeof(CMD_SIGNATURE)) == 0) {
      
        switch(buf[sizeof(CMD_SIGNATURE)] - '0'){

            case 0: //test communication
            send_cmd("BTLDCMD0 received");
            break;
    
            case 1: //reset pages
            reset_pages();
            break;

            case 2: //reset MCU
            int res = buf[sizeof(CMD_SIGNATURE) + 1];
            write_last_page(current_Page, res);
            Delay_ms(100);
            //ResetMCU();
            JumpToApplication(USER_CODE_OFFSET);
            
            break;
            
            case 3: // set entire page to 0xEE and advance
            
            reset_pages();
            int result;
            
            memset(pageData, 0xEE, FLASH_PAGE_SIZE); //sizeof(pageData)
            currentPageOffset = FLASH_PAGE_SIZE;
            result = write_page(current_Page);
            
            memset(pageData, 0xDD, FLASH_PAGE_SIZE); //sizeof(pageData)
            currentPageOffset = FLASH_PAGE_SIZE;
            result = write_page(current_Page);
            
            memset(pageData, 0xCC, FLASH_PAGE_SIZE); //sizeof(pageData)
            currentPageOffset = FLASH_PAGE_SIZE;
            result = write_page(current_Page);
            
            memset(pageData, 0xBB, FLASH_PAGE_SIZE); //sizeof(pageData)
            currentPageOffset = FLASH_PAGE_SIZE;
            result = write_page(current_Page);
            
            
            
            char buf2[64];
            count = sprintf(buf2, "BTLDRSP-%d", result);
            tud_cdc_n_write(0, buf2, count); 
            tud_cdc_n_write_flush(0);
            
            break;

        }      
    }
    else {
        memcpy(pageData + currentPageOffset, (const void *)buf, sizeof(buf));
        currentPageOffset += sizeof(buf);

        if (currentPageOffset != FLASH_PAGE_SIZE) return;
                    
        write_page(current_Page);

        send_cmd("BTLDCMD2");
          
    }
}


void send_cmd(const char* cmd){
   char buf[CDCACM_BUF_SIZE];
   int count = sprintf(buf, cmd);
   tud_cdc_n_write(0, buf, count); 
   tud_cdc_n_write_flush(0);
}    


int write_page(uint32_t currentPage){
    
    if (currentPageOffset == 0) return 0; //no write
    
    int status = Flash_ProgramFullPage(currentPage, pageData);
    
    if (status){
        return status;
    }
    
    current_Page++;
    currentPageOffset = 0;
    
    return 0;
}


int write_last_page(uint32_t currentPage, uint32_t res){
    
    if (currentPageOffset == 0) return 0; //no write
    
    uint32_t trim = res ? CDCACM_BUF_SIZE - res : 0;
    
    if (currentPageOffset < FLASH_PAGE_SIZE) { // in case of partial write empty rest of buffer
        memset(pageData + currentPageOffset - trim, 0xFF, sizeof(pageData) - currentPageOffset + trim);
    }
    
    return write_page(currentPage);
}









// Invoked when cdc when line state changed e.g connected/disconnected
// Use to reset to DFU when disconnect with 1200 bps
void tud_cdc_line_state_cb(uint8_t instance, bool dtr, bool rts) {
  (void)rts;

  // DTR = false is counted as disconnected
  if (!dtr) {
    // touch1200 only with first CDC instance (Serial)
    if (instance == 0) {
      cdc_line_coding_t coding;
      tud_cdc_get_line_coding(&coding);
      if (coding.bit_rate == 1200) {
        if (board_reset_to_bootloader) {
          board_reset_to_bootloader();
        }
      }
    }
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
