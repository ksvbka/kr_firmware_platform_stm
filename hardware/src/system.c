#include "system.h"
#include "stm32f0xx.h"
#include "typedef.h"

static volatile uint32_t g_count = 0;
static volatile uint32_t g_tick_count;
static volatile uint32_t g_second_count;

extern void SystemCoreClockUpdate (void);
static void set_clock_source(uint8_t clock_source);

void system_init(uint8_t clock_source) {
	/**
	 * Only config clock source, other config already called in SystemInit() in
	 * system_stm32f0xx.c
	 */
	set_clock_source(clock_source);

	/* Config SysTick count event each 1ms (F = 1000Hz)*/
	if (SysTick_Config(SystemCoreClock / 1000)) {
		/* Capture error */
		while (1);
	}

	/* Reset system tick*/
	g_tick_count = 0;
	g_second_count = 0;
}

void update_tick_count() {
	g_tick_count++;
	g_count++;
	if(g_count == 1000) {
		g_second_count++;
		g_count = 0;
	}
}

uint32_t get_tick_count() {
	return g_tick_count;
}

uint32_t get_second_count() {
	return g_second_count;
}

void delay_ms(uint32_t time_ms){
	uint32_t now = get_tick_count();
	uint32_t then = now + time_ms;
	while(then >= now){
		now = get_tick_count();
	}
}

void set_clock_source(uint8_t clock_source) {
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;
	switch (clock_source) {
	case CLOCK_SOURCE_HSI:
		FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

		/* HCLK = SYSCLK */
		RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

		/* PCLK = HCLK */
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;

		/* PLL configuration = (HSI/2) * 12 = ~48 MHz */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
		RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMULL12);

		/* Enable PLL */
		RCC->CR |= RCC_CR_PLLON;

		/* Wait till PLL is ready */
		while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
		}

		/* Select PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

		/* Wait till PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL) {
		}
		break;
	case  CLOCK_SOURCE_HSE:
		/* Enable HSE */
		RCC->CR |= ((uint32_t)RCC_CR_HSEON);
		break;
	case CLOCK_SOURCE_HSE_BYPASS:
		/* HSE oscillator bypassed with external clock */
		RCC->CR |= (uint32_t)(RCC_CR_HSEON | RCC_CR_HSEBYP);
		break;
	}


	do {
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
		HSEStatus = (uint32_t)0x01;
	} else {
		HSEStatus = (uint32_t)0x00;
	}

	if (HSEStatus == (uint32_t)0x01) {
		/* Enable Prefetch Buffer and set Flash Latency */
		FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

		/* HCLK = SYSCLK */
		RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

		/* PCLK = HCLK */
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;

		/* PLL configuration = HSE * 6 = 48 MHz */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
		RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMULL6);

		/* Enable PLL */
		RCC->CR |= RCC_CR_PLLON;

		/* Wait till PLL is ready */
		while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
		}

		/* Select PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

		/* Wait till PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL) {
		}
	} else {
		/* If HSE fails to start-up, the application will have wrong clock
			 configuration. User can add here some code to deal with this error */
	}

	SystemCoreClockUpdate();
}
