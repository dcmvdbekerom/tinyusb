#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/crs.h>
//#include <libopencm3/cm3/common.h>

// -----------------------------------------------------------------------------
// LED
#define LED_PORT              GPIOC
#define LED_PIN               GPIO6
#define LED_STATE_ON          0

// -----------------------------------------------------------------------------
// BUTTON
#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            GPIO13
#define BUTTON_STATE_ACTIVE   0

// -----------------------------------------------------------------------------
// UART2 (ST-Link VCP)
#define UART_DEV              USART2
#define UART_PORT             GPIOA
#define UART_TX_PIN           GPIO2
#define UART_RX_PIN           GPIO3


//#define PWR_CR1_VOS_MASK       (0x3 << 9)
#define PWR_CR1_VOS_SCALE_1    (0x1 << 9) // 0b01: Scale 1 mode
#define RCC_CR_HSI48ON     (1 << 22)
#define RCC_CR_HSI48RDY    (1 << 23)
#define RCC_CCIPR_CLK48SEL_SHIFT     8
#define RCC_CCIPR_CLK48SEL_MASK      (0x3 << RCC_CCIPR_CLK48SEL_SHIFT)
#define RCC_CCIPR_CLK48SEL_HSI48     (0x1 << RCC_CCIPR_CLK48SEL_SHIFT)
// -----------------------------------------------------------------------------
// Clock Setup
static inline void board_clock_init(void)
{
    // Enable power interface clock
    rcc_periph_clock_enable(RCC_PWR);
    // Set voltage scaling to Scale 1
    PWR_CR1 = (PWR_CR1 & ~PWR_CR1_VOS_MASK) | PWR_CR1_VOS_SCALE_1;
    // Enable HSI and wait until ready
    rcc_osc_on(RCC_HSI);
    rcc_wait_for_osc_ready(RCC_HSI);

    // Enable HSI48 for USB
    // rcc_osc_on(RCC_HSI48);
    // rcc_wait_for_osc_ready(RCC_HSI48);
    RCC_CR |= RCC_CR_HSI48ON;
    while ((RCC_CR & RCC_CR_HSI48RDY) == 0);

    // rcc_periph_clock_enable(RCC_CRS);
    // CRS_CFGR = CRS_CFGR_SYNCSRC_USB | CRS_CFGR_SYNCPOL_RISING | CRS_CFGR_RELOAD(48000) | CRS_CFGR_FELIM(34);
    // CRS_CR = CRS_CR_AUTOTRIMEN | CRS_CR_CEN;


    // Configure PLL: source = HSI16, PLLN = 8, PLLR = /2 (gives 64MHz)
    rcc_set_pll_source(RCC_PLLCFGR_PLLSRC_HSI16);
    rcc_set_main_pll(RCC_HSI, 
        RCC_PLLCFGR_PLLM_DIV(1),
        RCC_PLLCFGR_PLLN_MUL(8),
        RCC_PLLCFGR_PLLP_DIV(2),
        RCC_PLLCFGR_PLLQ_DIV(2),
        RCC_PLLCFGR_PLLR_DIV(2));

    //rcc_pll_enable();
    rcc_wait_for_osc_ready(RCC_PLL);

    // Configure flash latency
    //flash_set_ws(2); //default

    // Set SYSCLK source to PLL
    rcc_set_sysclk_source(RCC_CFGR_SWS_PLLRCLK);
    rcc_wait_for_sysclk_status(RCC_CFGR_SWS_PLLRCLK);

    // Set AHB and APB1 to no division
    rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
    rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

    // Setup USB clock to use HSI48
    RCC_CCIPR = (RCC_CCIPR & ~RCC_CCIPR_CLK48SEL_MASK) | RCC_CCIPR_CLK48SEL_HSI48;

    // Enable CRS (Clock Recovery System) for USB sync
    rcc_periph_clock_enable(RCC_CRS);

    CRS_CFGR = CRS_CFGR_SYNCSRC_USB | CRS_CFGR_SYNCPOL_RISING | CRS_CFGR_RELOAD(48000) | CRS_CFGR_FELIM(34);
    CRS_CR = CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
}

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */