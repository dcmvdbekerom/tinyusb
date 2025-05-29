/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2023 HiFiPhile
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
 * This file is part of the TinyUSB stack.
 */

/* metadata:
   manufacturer: STMicroelectronics
*/

//#include "stm32g0xx_hal.h"
#include "stm32g0xx.h"
#include "bsp/board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_UCPD1_2_IRQHandler(void) {
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
// #ifdef UART_DEV
// UART_HandleTypeDef UartHandle;
// #endif

void board_init(void) {
  //HAL_Init(); // required for HAL_RCC_Osc TODO check with freeRTOS
  board_clock_init();

  // Enable All GPIOs clocks


    RCC->IOPENR |= RCC_IOPENR_GPIOAEN
                 | RCC_IOPENR_GPIOBEN
                 | RCC_IOPENR_GPIOCEN
                 | RCC_IOPENR_GPIODEN
                 | RCC_IOPENR_GPIOEEN;

    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
    RCC->APBENR1 |= RCC_APBENR1_PWREN;

//DvdB: Old HAL code, may need it later if the CMSIS code turns out not to work
// #if CFG_TUSB_OS == OPT_OS_NONE
  // // 1ms tick timer
  // SysTick_Config(SystemCoreClock / 1000);
// #elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // // Explicitly disable systick to prevent its ISR runs before scheduler start
  // SysTick->CTRL &= ~1U;

  // // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  // NVIC_SetPriority(USB_UCPD1_2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
// #endif

    #if CFG_TUSB_OS == OPT_OS_NONE
      // 1ms tick timer using SysTick
      SysTick->LOAD  = (SystemCoreClock / 1000) - 1;
      SysTick->VAL   = 0;
      SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                       SysTick_CTRL_TICKINT_Msk   |
                       SysTick_CTRL_ENABLE_Msk;

    #elif CFG_TUSB_OS == OPT_OS_FREERTOS
      // Disable SysTick to prevent early interrupts before scheduler starts
      SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

      // Set USB interrupt priority
      NVIC_SetPriority(USB_UCPD1_2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    #endif


  //GPIO_InitTypeDef GPIO_InitStruct;

  // // LED
    uint32_t led_pin2 = LED_PIN * LED_PIN; //double the width for use with config registers that need 2bit per pin
    uint32_t led_msk2 = 0b11 * led_pin2;
    
    LED_PORT->MODER   = (LED_PORT->MODER   & (~led_msk2)) | (led_pin2 * 0b01);// Set mode to output (01)
    LED_PORT->OTYPER  = (LED_PORT->OTYPER  & ~(uint32_t)LED_PIN) | (LED_PIN * 0b0);// Set output type to push-pull (0)
    LED_PORT->OSPEEDR = (LED_PORT->OSPEEDR & (~led_msk2)) | (led_pin2 * 0b11);// Set speed to high (11)
    LED_PORT->PUPDR   = (LED_PORT->PUPDR   & (~led_msk2)) | (led_pin2 * 0b01);// Set pull-up (01)

  // GPIO_InitTypeDef GPIO_InitStruct;

  // // LED
  // GPIO_InitStruct.Pin = LED_PIN;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  // HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

   board_led_write(false);


    uint32_t button_pin2 = BUTTON_PIN * BUTTON_PIN;
    uint32_t button_msk2 = 0b11 * button_pin2;
    
     // Clear mode bits (2 bits per pin) and set to input (00)
    BUTTON_PORT->MODER = (BUTTON_PORT->MODER & ~button_msk2) | (button_pin2 * 0b00);
    
    // Set pull-up or pull-down depending on BUTTON_STATE_ACTIVE
    uint32_t pupd_val = (BUTTON_STATE_ACTIVE ? 0b10 : 0b01);
    BUTTON_PORT->PUPDR = (BUTTON_PORT->PUPDR & ~button_msk2) | (button_pin2 * pupd_val); 

  
//DvdB: Remove for now, could uncomment back in later 
// #ifdef UART_DEV
  // UART_CLK_EN();

  // // UART
  // GPIO_InitStruct.Pin       = UART_TX_PIN | UART_RX_PIN;
  // GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull      = GPIO_PULLUP;
  // GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  // GPIO_InitStruct.Alternate = UART_GPIO_AF;
  // HAL_GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);

  // UartHandle = (UART_HandleTypeDef){
    // .Instance        = UART_DEV,
    // .Init.BaudRate   = CFG_BOARD_UART_BAUDRATE,
    // .Init.WordLength = UART_WORDLENGTH_8B,
    // .Init.StopBits   = UART_STOPBITS_1,
    // .Init.Parity     = UART_PARITY_NONE,
    // .Init.HwFlowCtl  = UART_HWCONTROL_NONE,
    // .Init.Mode       = UART_MODE_TX_RX,
    // .Init.OverSampling = UART_OVERSAMPLING_16,
    // .AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT
  // };
  // HAL_UART_Init(&UartHandle);
// #endif

//DvdB: If it's optional, don't do it..
  // USB Pins TODO double check USB clock and pin setup
  // Configure USB DM and DP pins. This is optional, and maintained only for user guidance.
  // GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // /* Peripheral clock enable */
  // __HAL_RCC_USB_CLK_ENABLE();

  // /* Enable VDDUSB */
  // HAL_PWREx_EnableVddUSB();
  
  // Enable USB peripheral clock
  RCC->APBENR1 |= RCC_APBENR1_USBEN;

  // Enable VDDUSB
  PWR->CR2 |= PWR_CR2_USV;
      
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  if (state == LED_STATE_ON) {
    LED_PORT->BSRR = LED_PIN;           // Set pin
  } else {
    LED_PORT->BRR = LED_PIN;            // Reset pin
  }
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == ((BUTTON_PORT->IDR & BUTTON_PIN) ? 1U : 0U);
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;
  volatile uint32_t * stm32_uuid = (volatile uint32_t *) UID_BASE;
  uint32_t* id32 = (uint32_t*) (uintptr_t) id;
  uint8_t const len = 12;

  id32[0] = stm32_uuid[0];
  id32[1] = stm32_uuid[1];
  id32[2] = stm32_uuid[2];

  return len;
}
//DvdB: I disabled usart debugging, so this shouldn't be necessary..

// int board_uart_read(uint8_t *buf, int len) {
  // (void) buf;
  // (void) len;
  // return 0;
// }

// int board_uart_write(void const *buf, int len) {
// #ifdef UART_DEV
  // HAL_UART_Transmit(&UartHandle, (uint8_t*)(uintptr_t) buf, len, 0xffff);
  // return len;
// #else
  // (void) buf;
  // (void) len;
  // return 0;
// #endif
// }

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  system_ticks++;
  // HAL_IncTick();
}

uint32_t board_millis(void) {
  return system_ticks;
}
#endif

void HardFault_Handler(void) {
  __asm("BKPT #0\n");
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {

}
