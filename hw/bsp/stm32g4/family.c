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
 * This file is part of the TinyUSB stack.
 */

/* metadata:
   manufacturer: STMicroelectronics
*/

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_bus.h"

#include "bsp/board_api.h"
#include "board.h"

#ifdef UART_ID
  #if UART_ID == 1
    #define USARTn            USART1
    #define UARTn_CLK_ENABLE  __HAL_RCC_USART1_CLK_ENABLE
  #elif UART_ID == 2
    #define USARTn            USART2
    #define UARTn_CLK_ENABLE  __HAL_RCC_USART2_CLK_ENABLE
  #elif UART_ID == 3
    #define USARTn            USART3
    #define UARTn_CLK_ENABLE  __HAL_RCC_USART3_CLK_ENABLE
  #elif UART_ID == 11
    #define USARTn            LPUART1
    #define UARTn_CLK_ENABLE  __HAL_RCC_LPUART1_CLK_ENABLE
  #endif
#endif

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_HP_IRQHandler(void) {
  tud_int_handler(0);
}

void USB_LP_IRQHandler(void) {
  tud_int_handler(0);
}

// USB wakeup EXTI IRQ is not enabled by the fsdev driver (see fsdev_stm32.h);
// restore when STOP-mode wakeup is implemented.
//void USBWakeUp_IRQHandler(void) {
//  tud_int_handler(0);
//}

// USB PD
void UCPD1_IRQHandler(void) {
  tuc_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
#ifdef UART_ID
UART_HandleTypeDef UartHandle;
#endif

void board_init(void) {
  HAL_Init();
  board_clock_init();
  
#ifdef DAC_ENABLE
  DAC_init();
#endif

  OPAMP_Init();


  // Enable All GPIOs clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  
#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // Explicitly disable systick to prevent its ISR from running before scheduler start
  SysTick->CTRL &= ~1U;

  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB_HP_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_SetPriority(USB_LP_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  //NVIC_SetPriority(USBWakeUp_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

  GPIO_InitTypeDef GPIO_InitStruct;

  // LED
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  board_led_write(false);

  // Button
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = BUTTON_STATE_ACTIVE ? GPIO_PULLDOWN : GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

#ifdef UART_ID
  UARTn_CLK_ENABLE();

  // UART
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin       = UART_TX_PIN | UART_RX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = UART_GPIO_AF;
  HAL_GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);

  UartHandle = (UART_HandleTypeDef){
    .Instance        = USARTn,
    .Init.BaudRate   = CFG_BOARD_UART_BAUDRATE,
    .Init.WordLength = UART_WORDLENGTH_8B,
    .Init.StopBits   = UART_STOPBITS_1,
    .Init.Parity     = UART_PARITY_NONE,
    .Init.HwFlowCtl  = UART_HWCONTROL_NONE,
    .Init.Mode       = UART_MODE_TX_RX,
    .Init.OverSampling = UART_OVERSAMPLING_16
  };
  HAL_UART_Init(&UartHandle);
  HAL_UARTEx_EnableFifoMode(&UartHandle);
#endif

  // USB Pins TODO double check USB clock and pin setup
  // Configure USB DM and DP pins. This is optional, and maintained only for user guidance.
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_USB_CLK_ENABLE();

  board_vbus_sense_init();

#if 1
  // USB PD
  // Default CC1/CC2 is PB4/PB6

  // Enable pwr for disabling dead battery feature in Power's CR3
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_CRC_CLK_ENABLE();
  __HAL_RCC_UCPD1_CLK_ENABLE();

  // Enable DMA for USB PD
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
#endif

  GPIO_Output_Init();

}

void board_system_reset(void){
    HAL_NVIC_SystemReset();
}


#define SYSTEM_BOOTLOADER_ADDR   0x1FFF0000

void board_reset_to_bootloader(void) {
    // 1. Globally disable interrupts while we mess with context
    __disable_irq();

    // 2. Shut down your USB stack cleanly if using TinyUSB
    #if CFG_TUD_ENABLED
    tud_deinit(0);
    #endif

    // 3. Reset the STM32 RCC Clocks back to default 16MHz HSI state
    HAL_RCC_DeInit();

    // 4. Disable SysTick completely (ARM Cortex-M Syntax)
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // 5. Clear all active peripheral interrupt channels
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // 6. Re-enable interrupts right before jumping 
    // (The system bootloader relies on interrupts to run USB DFU)
    __enable_irq();

    // 7. ARM MANDATORY: Fetch and assign the Main Stack Pointer (MSP)
    __set_MSP(*(__IO uint32_t*)SYSTEM_BOOTLOADER_ADDR);

    // 8. Fetch the bootloader reset handler address (Base + 4 bytes)
    
    //   void (*bootloader_entry)(void) = (void (*)(void))0x1FFF8000;
//
//   bootloader_entry();
    
    uint32_t jump_address = *(__IO uint32_t*)(SYSTEM_BOOTLOADER_ADDR + 4);

    // 9. Execute the inline function pointer call to jump
    ((void (*)(void))jump_address)();

    // Catch loop just in case
    while(1);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  GPIO_PinState pin_state = (GPIO_PinState)(state ? LED_STATE_ON : (1 - LED_STATE_ON));
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, pin_state);
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
}

void board_set_select_01(void){
   
   LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7); //Select Hall (not VREF)
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

int board_uart_read(uint8_t *buf, int len) {
#ifdef UART_ID
  int count = 0;
  while (count < len) {
    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE)) {
      buf[count] = (uint8_t) UartHandle.Instance->RDR;
      count++;
    } else {
      break;
    }
  }
  return count;
#else
  (void) buf; (void) len;
  return 0;
#endif
}

int board_uart_write(void const *buf, int len) {
#ifdef UART_ID
  const uint8_t *p = (const uint8_t *) buf;
  int count = 0;
  while (count < len) {
    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE)) {
      UartHandle.Instance->TDR = p[count];
      count++;
    } else {
      break;
    }
  }
  return count;
#else
  (void) buf; (void) len;
  return -1;
#endif
}

#ifdef DAC_ENABLE
void DAC_set_values(uint32_t ch1, uint32_t ch2)
{
    /* 12-bit right-aligned data */
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, ch1&0x00000FFF);
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, ch2&0x00000FFF);

    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);
}
#endif



#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  HAL_IncTick();
  system_ticks++;
}

uint32_t tusb_time_millis_api(void) {
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
