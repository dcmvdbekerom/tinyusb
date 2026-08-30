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
static int parse_dac_command(char *buf, uint32_t *ch1, uint32_t *ch2);

/*------------- MAIN -------------*/
int main(void) {
  board_init();

  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  board_init_after_tusb();

  DAC_set_values( 0x800, 0x800);

  while (1) {
    tud_task(); // tinyusb device task
    cdc_task();
    led_blinking_task();
  }
}

// echo to either Serial0 or Serial1
// with Serial0 as all lower case, Serial1 as all upper case
static void echo_serial_port(uint8_t itf, uint8_t buf[], uint32_t count) {
  uint8_t const case_diff = 'a' - 'A';
  uint32_t ch1;
  uint32_t ch2;
  uint8_t ret_buf[] = "RESULT=X\r\n";

  int res = parse_dac_command((char*)buf, &ch1, &ch2);
  if (!res){
      if (ch1 < 0x1000 && ch2 < 0x1000){
        DAC_set_values(ch1, ch2);
      }
  }
  ret_buf[7] = (uint8_t)res + '0';
  for (uint32_t i=0; i<10; i++){
    tud_cdc_n_write_char(itf, ret_buf[i]);
  }
  
  for (uint32_t i = 0; i < count; i++) {
    if (itf == 0) {
      // echo back 1st port as lower case
      if (isupper(buf[i])) {
        buf[i] += case_diff;
      }
    } else {
      // echo back 2nd port as upper case
      if (islower(buf[i])) {
        buf[i] -= case_diff;
      }
    }

    tud_cdc_n_write_char(itf, buf[i]);
  }
  tud_cdc_n_write_flush(itf);
}

// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

static int parse_dac_command(char *buf, uint32_t *ch1, uint32_t *ch2)
{
    char *p;
    char *end;
    
    if (strncmp(buf, "DAC", 3) != 0) return 1;// Check command
    if (buf[3] != ' ' && buf[3] != '\0') return 2;// Make sure "DAC" is actually a token
    
    p = buf + 3;
    while (*p == ' ' || *p == '\t') p++;
    
    *ch1 = strtoul(p, &end, 10); // Parse channel 1
    if (end == p) return 3;       // no number
    p = end;
    while (*p == ' ' || *p == '\t') p++;// Skip whitespace

    *ch2 = strtoul(p, &end, 10); // Parse channel 2
    if (end == p) return 4;       // no number
    p = end;
    while (*p == ' ' || *p == '\t') p++;// Skip trailing whitespace
    // if (*p != '\0') return 5; // Nothing else is allowed
    
    return 0;
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
        echo_serial_port(0, buf, count);
        echo_serial_port(1, buf, count);
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
        board_reset_to_bootloader();
      }
    }
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  uint32_t now = 0;
  //static uint32_t counter = 0;
  static bool led_state = false;
 
  now = tusb_time_millis_api();

  //DAC_set_values( now, -now);

  // Blink every interval ms
  if (now - start_ms < blink_interval_ms) {
    return; // not enough time
  }
  
  start_ms = now;
  
  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
