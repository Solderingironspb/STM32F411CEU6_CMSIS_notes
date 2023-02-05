/**
 ******************************************************************************
 *  @file stm32f411xx_CMSIS.c
 *  @brief CMSIS на примере МК STM32F411CEU6
 *  @author Волков Олег
 *  @date 06.02.2023
 *
  ******************************************************************************
 * @attention
 *
 *  Библиотека помогает разобраться с библиотекой CMSIS на примере
 *  МК STM32F411CEU6
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md
 *  Группа ВК: https://vk.com/solderingiron.stm32
 *  Работал по Reference Manual: https://www.st.com/resource/en/reference_manual/dm00119316-stm32f411xc-e-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 *
 ******************************************************************************
 */

#include <stm32f411xx_CMSIS.h>

void CMSIS_Debug_init(void) {
	/*During and just after reset, the alternate functions are not active and the I / O ports are
	configured in input floating mode.
	The debug pins are in AF pull - up / pull - down after reset :
	• PA15: JTDI in pull-up
	• PA14: JTCK/SWCLK in pull-down
	• PA13: JTMS/SWDAT in pull-up
	• PB4: NJTRST in pull-up
	• PB3: JTDO in floating state
	*/
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); //Запуск тактирования порта A
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN); //Запуск тактирования порта B
	//Настроим режим Serial Wire
	//PA14: JTCK/SWCLK in pull-down
    //PA13: JTMS/SWDAT in pull-up
	//Остальное можем использовать по-своему усмотрению
	//PA15
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER15_Msk, 0b00 << GPIO_MODER_MODER15_Pos); //00: Input(reset state)
	CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD15); //00: No pull-up, pull-down
	//PB4
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER4_Msk, 0b00 << GPIO_MODER_MODER4_Pos); //00: Input(reset state)
	CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD4); //00: No pull-up, pull-down
	//PB3
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER3_Msk, 0b00 << GPIO_MODER_MODER3_Pos); //00: Input(reset state)
	CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD3); //00: No pull-up, pull-down
	CLEAR_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR3); //Просто сбросим изначальное состояние на 0.
	
	/*Заблокируем доступ для редактирования конфигурации PA13 и PA14*/
	GPIOA->LCKR = GPIO_LCKR_LCKK | GPIO_LCKR_LCK13 | GPIO_LCKR_LCK14;
	GPIOA->LCKR = GPIO_LCKR_LCK13 | GPIO_LCKR_LCK14;
	GPIOA->LCKR = GPIO_LCKR_LCKK | GPIO_LCKR_LCK13 | GPIO_LCKR_LCK14;
	GPIOA->LCKR;
	
}

void CMSIS_RCC_SystemClock_96MHz(void) {
	SET_BIT(RCC->CR, RCC_CR_HSION); //Запустим внутренний RC генератор на 16 МГц
	while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0) ; //Дождемся поднятия флага о готовности
	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP); //Просто сбросим этот бит в 0(Хотя изначально он и так должен быть в 0). 
	SET_BIT(RCC->CR, RCC_CR_HSEON); //Запустим внешний кварцевый резонатор. Он у нас на 25 MHz.
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0) ; //Дождемся поднятия флага о готовности
	SET_BIT(RCC->CR, RCC_CR_CSSON); //Включим CSS
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE); //Выберем HSE в качестве System Clock(PLL лучше пока не выбирать, он у нас отключен)
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_0); //0xxx: system clock not divided
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, RCC_CFGR_PPRE1_DIV2); //100: AHB clock divided by 2
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, RCC_CFGR_PPRE2_DIV1); //0xx: AHB clock not divided
	MODIFY_REG(RCC->CFGR, RCC_CFGR_RTCPRE_Msk, 25 << RCC_CFGR_RTCPRE_Pos); //Деление на 25
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO1_Msk, 0b11 << RCC_CFGR_MCO1_Pos); //11: PLL clock selected
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_I2SSRC); //0: PLLI2S clock used as I2S clock source
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO1PRE_Msk, RCC_CFGR_MCO1PRE_0); //0xx: no division
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, 0b010 << FLASH_ACR_LATENCY_Pos); //T0010: Two wait states 3 WS (4 CPU cycles) 90 < HCLK ≤ 100
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN); //Prefetch is enabled
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, 25 << RCC_PLLCFGR_PLLM_Pos); //Деление на 25
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, 192 << RCC_PLLCFGR_PLLN_Pos); //Умножение на 192
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_Msk, 0 << RCC_PLLCFGR_PLLP_Pos); // Без деления
	SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC); //1: HSE oscillator clock selected as PLL and PLLI2S clock entry
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_Msk, 4 << RCC_PLLCFGR_PLLQ_Pos); //Деление на 4
	SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) ; //Дождемся, пока PLL запустится
	SET_BIT(RCC->CR, RCC_CR_PLLI2SON); //Запустим PLL для I2S
	while (READ_BIT(RCC->CR, RCC_CR_PLLI2SRDY) == 0) ; //Дождемся, пока PLL запустится
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); //Выберем HSE в качестве System Clock(PLL лучше пока не выбирать, он у нас отключен)
}

void CMSIS_SysTick_Timer_init(void) {
	CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Выключим таймер для проведения настроек.
	SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk); //Разрешим прерывания по таймеру
	SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk); //Выберем без делителя. Будет 96MHz
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 95999 << SysTick_LOAD_RELOAD_Pos); //Настроим прерывание на частоту в 1 кГц(т.е. сработка будет каждую мс)
	MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, 95999 << SysTick_VAL_CURRENT_Pos); //Начнем считать с 95999
	SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Запускаем таймер
}


/**
 ***************************************************************************************
 *  @breif Настройка Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */

volatile uint32_t SysTimer_ms = 0; //Переменная, аналогичная HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //Счетчик для функции Delay_ms
volatile uint32_t Timeout_counter_ms = 0; //Переменная для таймаута функций

/**
 ******************************************************************************
 *  @breif Delay_ms
 *  @param   uint32_t Milliseconds - Длина задержки в миллисекундах
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
	Delay_counter_ms = Milliseconds;
	while (Delay_counter_ms != 0) ;
}

/**
 ******************************************************************************
 *  @breif Прерывание по флагу COUNTFLAG (см. п. 4.5.1 SysTick control and status register (STK_CTRL))
 *  Список векторов(прерываний) можно найти в файле startup_stm32f103c8tx.S
 ******************************************************************************
 */
void SysTick_Handler(void) {

	SysTimer_ms++;

	if (Delay_counter_ms) {
		Delay_counter_ms--;
	}
	if (Timeout_counter_ms) {
		Timeout_counter_ms--;
	}
}

void CMSIS_PC13_OUTPUT_Push_Pull_init(void) {
	/*Настроим PC13 на выход в режиме Push-pull*/
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN); //Запустим тактирование порта C
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER13_Msk, 0b01 << GPIO_MODER_MODER13_Pos); //01: General purpose output mode
	CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT13); //0: Output push-pull (reset state)
	MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED13_Msk, 0b11 << GPIO_OSPEEDR_OSPEED13_Pos); //High speed
}

void CMSIS_Blink_PC13(uint32_t ms) {
	GPIOC->BSRR = GPIO_BSRR_BR13;
	Delay_ms(ms);
	GPIOC->BSRR = GPIO_BSRR_BS13;
	Delay_ms(ms);
}