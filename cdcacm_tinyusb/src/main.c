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
#define CDCACM_READ_BUF_SIZE 64
#define CDCACM_WRITE_BUF_SIZE 64


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

static uint8_t pageData[FLASH_PAGE_SIZE];
static volatile uint32_t current_Page = 128; //beyond last page to prevent writes
static volatile uint16_t currentPageOffset = 0;
//uint16_t erase_page = 1;    


void reset_pages(void);
int write_page(uint32_t currentPage);
void send_cmd(const char* cmd);



/*------------- MAIN -------------*/
int main(void) {
  board_init();

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

    uint8_t buf[CDCACM_READ_BUF_SIZE];
    int count = tud_cdc_n_read(0, buf, sizeof(buf));
    
    if (count == 0) return;
    
    if (strncmp((char*) buf, (char*) CMD_SIGNATURE, sizeof(CMD_SIGNATURE)) == 0) {
      
        switch(buf[sizeof(CMD_SIGNATURE)] - '0'){

            case 0:
            send_cmd("HELLO");
            break;
    
            case 1:
            reset_pages();
            break;

            // case 2: //reset MCU
            // write_page(current_Page);
            // Delay_ms(100);
            // ResetMCU();
            // break;

        }      
    }
    // else {
        // memcpy(pageData + currentPageOffset, (const void *)buf, sizeof(buf));
        // currentPageOffset += sizeof(buf);

        // if (currentPageOffset != FLASH_PAGE_SIZE) return;
                    
        // write_page(current_Page);

        // send_cmd("BTLDCMD2");
          
    // }
}


void send_cmd(const char* cmd){
   char buf[CDCACM_WRITE_BUF_SIZE];
   int count = sprintf(buf, cmd);
   tud_cdc_n_write(0, buf, count); 
   tud_cdc_n_write_flush(0);
}    


int write_page(uint32_t currentPage){
    
    if (currentPageOffset == 0) return 0; //no write
    
    if (currentPageOffset < FLASH_PAGE_SIZE) { //fill remainder with zeros in case of partial write
        memset(pageData + currentPageOffset, 0, sizeof(pageData) - currentPageOffset);
    }
    
    int status = Flash_ProgramFullPage(currentPage, pageData);
    
    if (status){
        return status;
    }
    
    current_Page++;
    currentPageOffset = 0;
    
    return 0;
}

// /* USER CODE BEGIN 4 */
// void write_flash_sector(uint32_t currentPage) {
  // uint32_t pageAddress = FLASH_BASE + (currentPage * SECTOR_SIZE);
  // uint32_t SectorError;

  // HAL_GPIO_WritePin(LED_1_PORT, LED_1_PIN, GPIO_PIN_SET);	
  // FLASH_EraseInitTypeDef EraseInit;
  // HAL_FLASH_Unlock();
                                  

  // /* Sector to the erase the flash memory (16, 32, 48 ... kbytes) */
  // if ((currentPage == 16) || (currentPage == 32) ||
      // (currentPage == 48) || (currentPage == 64) ||
      // (currentPage % 128 == 0)) {
    // EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    // EraseInit.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

    // /* Specify sector number. Starts from 0x08004000 */
    // EraseInit.Sector = erase_page++;
                                              
  

    // /* This is also important! */
    // EraseInit.NbSectors = 1;
    // HAL_FLASHEx_Erase(&EraseInit, &SectorError);
  // }

  // uint32_t dat;
  // for (int i = 0; i < SECTOR_SIZE; i += 4) {
    // dat = pageData[i+3];
    // dat <<= 8;
    // dat += pageData[i+2];
    // dat <<= 8;
    // dat += pageData[i+1];
    // dat <<= 8;
    // dat += pageData[i];
    // HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAddress + i, dat);
  // }
  // HAL_GPIO_WritePin(LED_1_PORT, LED_1_PIN,GPIO_PIN_RESET);  
  // HAL_FLASH_Lock();
// }











// static void cdc_task(void) {
  // uint8_t itf;

  // for (itf = 0; itf < CFG_TUD_CDC; itf++) {
    // // connected() check for DTR bit
    // // Most but not all terminal client set this when making connection
    // // if ( tud_cdc_n_connected(itf) )
    // {
      // if (tud_cdc_n_available(itf)) {
        // uint8_t buf[64];

        // uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

        // // echo back to both serial ports
        // echo_serial_port(0, buf, count);
        // echo_serial_port(1, buf, count);
      // }
    // }
  // }
// }



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
