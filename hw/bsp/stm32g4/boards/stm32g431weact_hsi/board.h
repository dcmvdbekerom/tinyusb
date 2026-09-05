/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020, Ha Thach (tinyusb.org)
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
   name: STM32 G431 WeAct
   url: https://www.st.com/en/evaluation-tools/nucleo-g474re.html
*/

#ifndef BOARD_H_
#define BOARD_H_


#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_bus.h"

// G474RE Nucleo does not has usb connection. We need to manually connect
// - PA12 for D+, CN10.12
// - PA11 for D-, CN10.14

// LED
#define LED_PORT              GPIOA
#define LED_PIN               GPIO_PIN_0
#define LED_STATE_ON          0

// Button
#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            GPIO_PIN_13
#define BUTTON_STATE_ACTIVE   1


// DAC
#define DAC_ENABLE

// // UART Enable for STLink VCOM
// #define UART_ID               11
// #define UART_GPIO_PORT        GPIOA
// #define UART_GPIO_AF          GPIO_AF12_LPUART1
// #define UART_TX_PIN           GPIO_PIN_2
// #define UART_RX_PIN           GPIO_PIN_3


//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+

static inline void board_clock_init(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  // Configure the main internal regulator output voltage
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  // Initializes the CPU, AHB and APB buses clocks
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT + 1;
  RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN       = 85;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) ;

#if 1 // TODO need to check if USB clock is enabled
  /* Enable HSI48 */
  memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /*Enable CRS Clock*/
  RCC_CRSInitTypeDef RCC_CRSInitStruct= {0};
  __HAL_RCC_CRS_CLK_ENABLE();

  /* Default Synchro Signal division factor (not divided) */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;

  /* Set the SYNCSRC[1:0] bits according to CRS_Source value */
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;

  /* HSI48 is synchronized with USB SOF at 1KHz rate */
  RCC_CRSInitStruct.ReloadValue =  __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;

  /* Set the TRIM[5:0] to the default value */
  RCC_CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

  /* Start automatic synchronization */
  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
#endif

    // // Enable MCO on pin A8:
    // __HAL_RCC_GPIOA_CLK_ENABLE();

    // GPIO_InitTypeDef GPIO_InitStruct = {0};
    // GPIO_InitStruct.Pin = GPIO_PIN_8;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // Crucial for 170MHz G4 clock speeds
    // GPIO_InitStruct.Alternate = GPIO_AF0_MCO;         // AF0 maps to MCO on STM32G4
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // // HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_4); 
    // HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); 

}



static inline void DAC_init(void)
{
    /* Enable clocks */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);

    /* DAC outputs:
       DAC1_CH1 -> PA4
       DAC1_CH2 -> PA5
    */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);

    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);

    /* Channel 1 */
    LL_DAC_SetTriggerSource(DAC1, LL_DAC_CHANNEL_1,
                            LL_DAC_TRIG_SOFTWARE);
    LL_DAC_SetOutputBuffer(DAC1, LL_DAC_CHANNEL_1,
                           LL_DAC_OUTPUT_BUFFER_ENABLE);

    /* Channel 2 */
    LL_DAC_SetTriggerSource(DAC1, LL_DAC_CHANNEL_2,
                            LL_DAC_TRIG_SOFTWARE);
    LL_DAC_SetOutputBuffer(DAC1, LL_DAC_CHANNEL_2,
                           LL_DAC_OUTPUT_BUFFER_ENABLE);

    /* Enable both channels */
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);
}

static void OPAMP_Init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    ////OPAMP1:
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    
    //OPAMP1_VINP0
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
    

    //OPAMP1_VOUT
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);
    
    LL_OPAMP_SetPowerMode(OPAMP1, LL_OPAMP_POWERMODE_NORMAL);
    LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_FOLLOWER);
    LL_OPAMP_SetInputNonInverting(OPAMP1, LL_OPAMP_INPUT_NONINVERT_IO0);
    LL_OPAMP_SetInternalOutput(OPAMP1, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
    LL_OPAMP_Enable(OPAMP1);
    
    ////OPAMP2:
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    
    //OPAMP2_VOUT
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);

    //OPAMP2_VINP2
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);

    LL_OPAMP_SetPowerMode(OPAMP2, LL_OPAMP_POWERMODE_NORMAL);
    LL_OPAMP_SetFunctionalMode(OPAMP2, LL_OPAMP_MODE_FOLLOWER);
    LL_OPAMP_SetInputNonInverting(OPAMP2, LL_OPAMP_INPUT_NONINVERT_IO2);
    LL_OPAMP_SetInternalOutput(OPAMP2, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
    LL_OPAMP_Enable(OPAMP2);
   
}

#define  SIG_PINS_CH1 (LL_GPIO_PIN_4 | LL_GPIO_PIN_6) //B
#define  SIG_PINS_CH2 (LL_GPIO_PIN_7 | LL_GPIO_PIN_8) //B
#define GAIN_PINS_CH2 (LL_GPIO_PIN_8 | LL_GPIO_PIN_9) //A

static void GPIO_Output_Init(void)
{
    /* Enable GPIOA and GPIOB clocks */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    /* Set output levels LOW before enabling output mode */
    LL_GPIO_ResetOutputPin(GPIOA, GAIN_PINS_CH2);
    LL_GPIO_ResetOutputPin(GPIOB, SIG_PINS_CH1|SIG_PINS_CH2);

    LL_GPIO_InitTypeDef gpio_init = {0};
    
    gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
    gpio_init.Speed = LL_GPIO_SPEED_LOW;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;
  
    gpio_init.Pin = GAIN_PINS_CH2;
    LL_GPIO_Init(GPIOA, &gpio_init);
    
    gpio_init.Pin = SIG_PINS_CH1|SIG_PINS_CH2;
    LL_GPIO_Init(GPIOB, &gpio_init);
}






static inline void board_vbus_sense_init(void)
{
  // Enable VBUS sense (B device) via pin PA9
}

#ifdef __cplusplus
 }
#endif

#endif /* BOARD_H_ */
