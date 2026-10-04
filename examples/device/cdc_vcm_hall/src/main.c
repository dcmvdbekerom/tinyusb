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
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "bsp/board_api.h"
#include "tusb.h"

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

enum {
  CMD_SUCCESS,
  CMD_ERROR_CMD_MISSING,
  CMD_ERROR_PARSE_VALUE
};
  
#define CMD_START_ACQUISITION       0x55AA55AA
#define CMD_STOP_ACQUISITION        0x00FF00FF

#define PING_PONG_BUF_SIZE    1024
#define PING_PONG_HALF_SIZE   (PING_PONG_BUF_SIZE / 2)


volatile uint8_t DMA_transfer_interrupt_flag = 0; 
uint8_t ping_pong_buffer[PING_PONG_BUF_SIZE];

static uint32_t bytes_written = 0;
static uint8_t *current_source_ptr = NULL;
static bool is_acquiring = false;
static bool send_zlp = false;  


static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

// static void led_blinking_task(void);
static void cdc_task(void);
static void vendor_task(void);
static int parse_command(char *buf);
//static int parse_dac_command(char *buf, uint32_t *ch1, uint32_t *ch2);

#define DMA_BUF_SIZE 1024
uint32_t timestamp_buffer[DMA_BUF_SIZE];

#define MAGIC_DFU_NUMBER   0xB00470AD
uint32_t dfu_flag __attribute__((persistent)) = 0; //this register is initialized randomly, with a tiny chance it is 0xB0047OAD, we accept this.

/*------------- MAIN -------------*/
int main(void) {
    
    // Check if the VCP app set the magic DFU flag
    if (dfu_flag == MAGIC_DFU_NUMBER) {
        dfu_flag = 0;
        board_reset_to_bootloader();
    }  

  board_init();

  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  board_init_after_tusb();

  select_signal_gain_ch1(0x8001, 0x8007); //Hall Front; 200x
  select_signal_gain_ch2(0x8001, 0x8002); //Hall Front; 200x
  
  board_init_DMA(ping_pong_buffer, PING_PONG_BUF_SIZE / sizeof(uint16_t));

  while (1) {
    tud_task(); // tinyusb device task
    cdc_task();
    vendor_task();
    // led_blinking_task();
  }
}

// echo to either Serial0 or Serial1
// with Serial0 as all lower case, Serial1 as all upper case
static void echo_serial_port(char* buf, uint32_t count) {
  //uint8_t const case_diff = 'a' - 'A';
  //uint32_t ch1;
  //uint32_t ch2;
  //uint8_t ret_buf[] = "RESULT=X\r\n";
  (void)count;
  
  parse_command((char*)buf);
  



  // int res = parse_dac_command((char*)buf, &ch1, &ch2);
  // if (!res){
      // if (ch1 < 0x1000 && ch2 < 0x1000){
        // DAC_set_values(ch1, ch2);
      // }
  // }
  // ret_buf[7] = (uint8_t)res + '0';
  // tud_cdc_n_write(0, buf, count);
  // tud_cdc_n_write(0, ret_buf, sizeof(ret_buf));
  // tud_cdc_n_write_flush(0);
}

// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

static inline char *get_token( char *input, char *token, size_t token_size)
{
    // Skip leading whitespace
    while (*input == ' ' || *input == '\t' ||
           *input == '\r' || *input == '\n') {
        input++;
    }
    // Copy token
    size_t i = 0;
    token[0] = '\0';
    while (*input != '\0' &&
           *input != ' ' && *input != '\t' &&
           *input != '\r' && *input != '\n') {

        if (i < token_size - 1) {
            token[i++] = *input;
        }
        input++;
    }
    token[i] = '\0';

    return input;
}

static void vcp_write(const char* buf){
    tud_cdc_n_write(0, buf, strlen(buf));
    tud_cdc_n_write(0, "\r\n",2);
    tud_cdc_n_write_flush(0);
}


static int parse_channel_values(char* buf, uint16_t* ch1, uint16_t* ch2){
   char token1[16], token2[16];
   char *end;
  
   buf = get_token(buf, token1, sizeof(token1));
   buf = get_token(buf, token2, sizeof(token2));   
   
   if (strcmp(token1, "CH1") == 0){
     *ch1 = (uint16_t)strtoul(token2, &end, 0);
     if (end == token2) {
        return 1; //unable to parse token 1
     }
     *ch1 |=  0x8000;
     *ch2 = 0x0;
   }
   else if (strcmp(token1, "CH2") == 0){
     *ch2 = (uint16_t)strtoul(token2, &end, 0);
     if (end == token2) {
        return 1;  //unable to parse token 1
     }
     *ch1 = 0x0;
     *ch2 |=  0x8000;
   }
   else{
     *ch1 = (uint16_t)strtoul(token1, &end, 0);
     if (end == token1) {
        return 1;  //unable to parse token 1
     }
     *ch2 = (uint16_t)strtoul(token2, &end, 0);
     if (end == token2) {
        return 2;  //unable to parse token 2
     }
     *ch1 |= 0x8000;
     *ch2 |= 0x8000;     
   }
   return 0;
}
    

static int parse_command(char *buf){
    //const char cmd_success[] = "SUCCESS!";
    //const char cmd_error_cmd_missing[] = "ERROR - 'CMD' MISSING";
    char token[16];
    uint16_t val1, val2;
    int res;  
      
    buf = get_token(buf, token, sizeof(token));
    if (strcmp(token, "CMD") != 0){
      vcp_write("ERROR - 'CMD' MISSING");
      return CMD_ERROR_CMD_MISSING;
    }
    
    buf = get_token(buf, token, sizeof(token));
    if (strcmp(token, "BTLD") == 0){
      vcp_write("STARTING BOOTLOADER");
      
      dfu_flag = MAGIC_DFU_NUMBER;
      board_system_reset();
      //board_reset_to_bootloader();
      
      return CMD_SUCCESS; //never get here
    }
    else if (strcmp(token, "DAC") == 0){
      //successfully read value for CH1
      res = parse_channel_values(buf, &val1, &val2);
      if (res) return CMD_ERROR_PARSE_VALUE;

      DAC_set_values(val1, val2);
      
      vcp_write("SET DAC SUCCESS!");
    }
    else if (strcmp(token, "SIG") == 0){
      res = parse_channel_values(buf, &val1, &val2);
      if (res) return CMD_ERROR_PARSE_VALUE;
      select_signal_gain_ch1(val1, 0);
      select_signal_gain_ch2(val2, 0);

      vcp_write("SET SIGNAL SUCCESS!");
    }
    
    else if (strcmp(token, "GAIN") == 0){
      res = parse_channel_values(buf, &val1, &val2);
      if (res) return CMD_ERROR_PARSE_VALUE;
      select_signal_gain_ch1(0, val1);
      select_signal_gain_ch2(0, val2);

      vcp_write("SET GAIN SUCCESS!");
    }
    
    else if (strcmp(token, "STREAM") == 0){
      vcp_write("STREAM OPTIONS");
    }
    else{
      vcp_write("UNKNOWN COMMAND");  
    }
    return CMD_SUCCESS;
}




//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
static void cdc_task(void) {
  for (uint8_t itf = 0; itf < CFG_TUD_CDC; itf++) {
    // connected() check for DTR bit
    // Most but not all terminal client set this when making connection
    // if ( tud_cdc_n_connected(itf) )
    {
      if (tud_cdc_n_available(itf)) {
        uint8_t buf[64];
        uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

        // echo back to both serial ports
        echo_serial_port((char*)buf, count);
        //echo_serial_port(1, buf, count);
      }

      // Press on-board button to send Uart status notification
      static uint32_t btn_prev = 0;
      static cdc_notify_uart_state_t uart_state = { .value = 0 };
      const uint32_t btn = board_button_read();
      if (!btn_prev && btn) {
        uart_state.dsr ^= 1;
        tud_cdc_notify_uart_state(&uart_state);
      }
      btn_prev = btn;
    }
  }
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
        vcp_write("STARTING BOOTLOADER");
        board_reset_to_bootloader();
      }
    }
  }
}


//-------------------------------------+
// VENDOR TASK
//-------------------------------------+



void vendor_task(void)
{
    if ( !tud_vendor_mounted() ) 
    {
        return;
    }

    // ==========================================
    // 1. READ RX FIFO FOR HOST COMMANDS
    // ==========================================
    uint32_t rx_available = tud_vendor_available();
    if (rx_available >= 4) // Commands are 4 bytes long
    {
        uint32_t command_received = 0;
        tud_vendor_read(&command_received, 4);
        
        // board_led_write(0);
        board_write_SPI((uint8_t*)&command_received, 4);
        // board_led_write(1);
        
        if (command_received == CMD_START_ACQUISITION)
        {
            board_start_acquisition();
            // board_led_write(1);
            is_acquiring = true;
            send_zlp = false;
            // Reset streaming tracking variables
            current_source_ptr = NULL;
            bytes_written = 0;
            DMA_transfer_interrupt_flag = 0; 
        }
        else if (command_received == CMD_STOP_ACQUISITION)
        {
            board_stop_acquisition();
            // board_led_write(0);
            is_acquiring = false;
            send_zlp = true; // Flag that we need to send a final ZLP
        }
    }

    // ==========================================
    // 2. HANDLE LAST TRANSFER TERMINATION (ZLP)
    // ==========================================
    if (!is_acquiring && send_zlp)
    {
        // Wait until all previous data has completely cleared out of the FIFO
        if (tud_vendor_write_available() == CFG_TUD_VENDOR_TX_BUFSIZE)
        {
            // Flushing an empty buffer forces TinyUSB to send a Zero-Length Packet (ZLP) [2]
            tud_vendor_write_flush(); 
            send_zlp = false; 
            current_source_ptr = NULL;
        }
        return; // Skip data streaming logic since acquisition is stopped
    }

    // If acquisition is disabled and ZLP is already handled, do not stream data
    if (!is_acquiring)
    {
        return;
    }

    // ==========================================
    // 3. STREAM TX DATA (Only if acquiring)
    // ==========================================
    
    // Capture new DMA events only if we aren't currently busy with a previous half
    if (current_source_ptr == NULL) 
    {
        if (DMA_transfer_interrupt_flag == 1) 
        {
            current_source_ptr = &ping_pong_buffer[0]; // First half ready
            DMA_transfer_interrupt_flag = 0;
            bytes_written = 0;
        } 
        else if (DMA_transfer_interrupt_flag == 2) 
        {
            current_source_ptr = &ping_pong_buffer[PING_PONG_HALF_SIZE]; // Second half ready
            DMA_transfer_interrupt_flag = 0;
            bytes_written = 0;
        }
    }

    // Stream from the active DMA half-buffer into TinyUSB's FIFO
    if (current_source_ptr != NULL) 
    {
        uint32_t available_space = tud_vendor_write_available();
        
        if (available_space > 0) 
        {
            uint32_t chunk_size = PING_PONG_HALF_SIZE - bytes_written;
            if (chunk_size > available_space) 
            {
                chunk_size = available_space;
            }
            
            tud_vendor_write(&current_source_ptr[bytes_written], chunk_size);
            bytes_written += chunk_size;
        }
        
        // Once the entire half-buffer has been pushed to the TinyUSB FIFO, release it
        if (bytes_written >= PING_PONG_HALF_SIZE) 
        {
            tud_vendor_write_flush(); // Force packet transmission on the bus
            current_source_ptr = NULL; // Ready to receive the next DMA half-buffer event
        }
    }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
// void led_blinking_task(void) {
  // static uint32_t start_ms = 0;
  // uint32_t now = 0;
  // //static uint32_t counter = 0;
  // static bool led_state = false;
 
  // now = tusb_time_millis_api();

  // //DAC_set_values( now, -now);

  // // Blink every interval ms
  // if (now - start_ms < blink_interval_ms) {
    // return; // not enough time
  // }
  
  // start_ms = now;
  
  // board_led_write(led_state);
  // led_state = 1 - led_state; // toggle
// }
