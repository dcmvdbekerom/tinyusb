/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020, Ha Thach (tinyusb.org)
 * Copyright (c) 2023, HiFiPhile
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
   name: STM32 G0B1 Nucleo
   url: https://www.st.com/en/evaluation-tools/nucleo-g0b1re.html
*/

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

// G0B1RE Nucleo does not has usb connection. We need to manually connect
// - PA12 for D+, CN10.12
// - PA11 for D-, CN10.14

// LED
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */

#define LED_PORT              GPIOC
#define LED_PIN               GPIO_PIN_6
#define LED_STATE_ON          0

// Button
#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            GPIO_PIN_13
#define BUTTON_STATE_ACTIVE   0

// UART Enable for STLink VCOM
#define UART_DEV              USART2
#define UART_CLK_EN           __HAL_RCC_USART2_CLK_ENABLE
#define UART_GPIO_PORT        GPIOA
#define UART_GPIO_AF          GPIO_AF1_USART2
#define UART_TX_PIN           GPIO_PIN_2
#define UART_RX_PIN           GPIO_PIN_3


//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
#if 1
// Clock configure for STM32G0B1RE Nucleo
static inline void board_clock_init(void)
{

  /** Configure the main internal regulator output voltage */
  //HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | ( PWR_CR1_VOS_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure. */
  // RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  // RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  // RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  // RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  // RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  // RCC_OscInitStruct.PLL.PLLN = 8;
  // RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  // RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  // RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  // HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
      
      // 1. Enable HSI oscillator
    RCC->CR |= RCC_CR_HSION;                   // Turn on HSI
    while ((RCC->CR & RCC_CR_HSIRDY) == 0){};   // Wait until HSI ready

    // 2. Configure HSI divider (HSIDIV bits 7:6 in RCC_CR)
    RCC->CR &= ~RCC_CR_HSIDIV;                 // Clear HSIDIV bits
    RCC->CR |= RCC_CR_HSIDIV_0;                // HSIDIV = DIV1 (0b00, so could omit)
    
    
#define RCC_HSICALIBRATION_DEFAULT     64U  

    // 3. HSI calibration (bits 15:8 in RCC_CR)
    RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_HSICAL) | RCC_HSICALIBRATION_DEFAULT;

    // 4. Configure PLL
    // Disable PLL first
    RCC->CR &= ~RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) != 0){};   // Wait until PLL disabled

    // Set PLL source to HSI (bit PLLSRC in RCC_PLLCFGR)
        RCC->PLLCFGR = (0 << RCC_PLLCFGR_PLLM_Pos)    // PLLM = DIV1 (0 means DIV1)
             | (8 << RCC_PLLCFGR_PLLN_Pos)    // PLLN = 8
             | (0 << RCC_PLLCFGR_PLLP_Pos)    // PLLP = DIV2 (0 means DIV2)
             | (0 << RCC_PLLCFGR_PLLQ_Pos)    // PLLQ = DIV2 (0 means DIV2)
             | (0 << RCC_PLLCFGR_PLLR_Pos)    // PLLR = DIV2 (0 means DIV2)
             | RCC_PLLCFGR_PLLSRC_HSI          // PLL source = HSI
             | RCC_PLLCFGR_PLLREN               // Enable PLLR output (usually needed)
             ;
    // 5. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0){};   // Wait until PLL ready
      
  

  // /** Initializes the CPU, AHB and APB buses clocks */
  // RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  // RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  // RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  // RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  // HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  
   // __asm("BKPT #4\n");


#define FLASH_LATENCY_2                 FLASH_ACR_LATENCY_1
// Configure Flash latency (FLASH_ACR register)
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_LATENCY_2;      // 2 wait states
    
    while( (FLASH->ACR&FLASH_ACR_LATENCY) != FLASH_LATENCY_2) {};

#define RCC_SYSCLK_DIV1                0x00000000U 
#define RCC_HCLK_DIV1                  0x00000000U  
 
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE))
        | RCC_CFGR_SW_PLLRCLK  // SYSCLK = PLL
        | RCC_SYSCLK_DIV1      // AHB prescaler = /1 //HAL macro
        | RCC_HCLK_DIV1 ;     // APB1 prescaler = /1 //HAL macro

    // Wait until PLL is used as system clock
    while ((RCC->CFGR & RCC_CFGR_SW) != RCC_CFGR_SW_PLLRCLK) { }


  // Configure CRS clock source
  // __HAL_RCC_CRS_CLK_ENABLE();
  // RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};
  // RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  // RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  // RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  // RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  // RCC_CRSInitStruct.ErrorLimitValue = 34;
  // RCC_CRSInitStruct.HSI48CalibrationValue = 32;
  //HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
  
  
    // Enable CRS peripheral clock
    RCC->APBENR1 |= RCC_APBENR1_CRSEN;

    // Build CFGR value
    CRS->CFGR = (47999 << CRS_CFGR_RELOAD_Pos)     |  // RELOAD value (48MHz / 1kHz - 1)
                (34 << CRS_CFGR_FELIM_Pos)         |  // Error limit
                (0 << CRS_CFGR_SYNCDIV_Pos)        |  // SYNC_DIV1
                (1 << CRS_CFGR_SYNCSRC_Pos)        |  // SYNC_SOURCE_USB
                (0 << CRS_CFGR_SYNCPOL_Pos);          // Rising polarity

    // Set HSI48 calibration value
    CRS->CR &= ~CRS_CR_TRIM_Msk;
    CRS->CR |= (32 << CRS_CR_TRIM_Pos);

    // Enable automatic trimming and frequency error counter
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;



  /* Select HSI48 as USB clock source */
  // RCC_PeriphCLKInitTypeDef usb_clk = {0 };
  // usb_clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
  // usb_clk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  // HAL_RCCEx_PeriphCLKConfig(&usb_clk);

    RCC->CCIPR2 = (RCC->CCIPR2 & ~RCC_CCIPR2_USBSEL_Msk) | (0b00 << RCC_CCIPR2_USBSEL_Pos);  // 00: HSI48 selected as USB clock

  // Enable HSI48
  // RCC_OscInitTypeDef osc_hsi48 = {0};
  // osc_hsi48.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  // osc_hsi48.HSI48State = RCC_HSI48_ON;
  // HAL_RCC_OscConfig(&osc_hsi48);
  
  
  RCC->CR |= RCC_CR_HSI48ON;

    // Wait until HSI48 is ready
    while ((RCC->CR & RCC_CR_HSI48RDY) == 0) {};
}
// DvdB: uncomment code that doesnt do anything
// #else

// // Clock configure for STM32G0 nucleo with B0 mcu variant for someone that is skilled enough
// // to rework and solder the B0 chip. Note: SB17 may need to be soldered as well (check user manual)
// static inline void board_clock_init(void)
// {
  // RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  // RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  // RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct = { 0 };

  // /** Configure the main internal regulator output voltage */
  // HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  // /** Initializes the RCC Oscillators according to the specified parameters
  // * in the RCC_OscInitTypeDef structure. */
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  // RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  // RCC_OscInitStruct.PLL.PLLN = 12;
  // RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  // RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  // RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  // HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // /* Select HSI48 as USB clock source */
  // PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  // PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  // HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  // /** Initializes the CPU, AHB and APB buses clocks */
  // RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              // |RCC_CLOCKTYPE_PCLK1;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  // RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  // RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  // HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
// }
#endif

#ifdef __cplusplus
 }
#endif

#endif /* BOARD_H_ */
