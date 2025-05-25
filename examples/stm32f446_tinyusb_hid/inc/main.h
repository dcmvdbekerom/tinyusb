#include "stm32f446xx.h"
#include <stdbool.h>

#ifndef _MAIN_H_
#define _MAIN_H_


static volatile uint32_t ms_ticks = 0;

uint32_t board_usb_get_serial(uint16_t *buf, uint32_t bufsize);

void SysTick_Handler(void)
{
    ms_ticks++;
}

uint32_t board_millis(void)
{
    return ms_ticks;
}

void board_init(void)
{
    // Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure PB2 as general purpose output
    GPIOB->MODER &= ~GPIO_MODER_MODE2_Msk;
    GPIOB->MODER |= (GPIO_MODER_MODE2_0); // Output mode

    GPIOB->OTYPER &= ~GPIO_OTYPER_OT2; // Push-pull
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2; // High speed
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD2; // No pull-up/pull-down

    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA11, PA12 as alternate function 10 (USB OTG FS)
    GPIOA->MODER &= ~(GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE12_Msk);
    GPIOA->MODER |= (GPIO_MODER_MODE11_1 | GPIO_MODER_MODE12_1); // AF mode

    GPIOA->AFR[1] &= ~((GPIO_AFRH_AFSEL11_Msk) | (GPIO_AFRH_AFSEL12_Msk));
    GPIOA->AFR[1] |= ((10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos));

    // Enable USB OTG FS clock
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

    // Configure SysTick for 1ms interrupts at 180 MHz core clock
    SysTick_Config(180000);

    NVIC_EnableIRQ(SysTick_IRQn);
}

void board_init_after_tusb(void)
{
    // Optional; do nothing
}

void board_led_write(bool on)
{
    if(on)
        GPIOB->BSRR = GPIO_BSRR_BS2; // Set PB2
    else
        GPIOB->BSRR = GPIO_BSRR_BR2; // Reset PB2
}

#endif /* _MAIN_H_ */