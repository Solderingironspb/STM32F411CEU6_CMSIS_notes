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
    /*Настройки:
     * Входной кварцевый резонатор 25 MHz
     * CSS Enabled
     * SYSCLK 96 MHz
     * AHB 96 MHz
     * PCLK1 48 MHz
     * PCLK2 96 MHz
     * USB 48 MHz
     * I2S Clock 48 MHz
     * */
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
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_Msk, 0b00 << RCC_PLLCFGR_PLLP_Pos); // 00: PLLP = 2
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC); //1: HSE oscillator clock selected as PLL and PLLI2S clock entry
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_Msk, 4 << RCC_PLLCFGR_PLLQ_Pos); //Деление на 4
    SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) ; //Дождемся, пока PLL запустится
    MODIFY_REG(RCC->PLLI2SCFGR, RCC_PLLI2SCFGR_PLLI2SM_Msk, 25 << RCC_PLLI2SCFGR_PLLI2SM_Pos); //Деление на 25
    MODIFY_REG(RCC->PLLI2SCFGR, RCC_PLLCFGR_PLLN_Msk, 192 << RCC_PLLCFGR_PLLN_Pos); //Умножение на 192
    MODIFY_REG(RCC->PLLI2SCFGR, RCC_PLLCFGR_PLLP_Msk, 4 << RCC_PLLCFGR_PLLP_Pos); //Деление на 4
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


void CMSIS_SPI1_init(void) {
    /*Настройка GPIO*/
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN); //Включение тактирования SPI1
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); //Включение тактирования порта А
    /*Какие ножки:*/
    //PA4 - NSS
    //PA5 - SCK
    //PA6 - MISO
    //PA7 - MOSI
    //Мы будем настраивать SPI в режим Master
    /*
     * SPIx_SCK  Master - Alternate function push-pull
     * SPIx_MOSI:
     *             Full duplex / master - Alternate function push-pull
     *             Simplex bidirectional data wire / master - Alternate function push-pull
     * SPIx_MISO:
     *             Full duplex / master - Input floating / Input pull-up
     *
     * SPIx_NSS:
     *             Hardware master /slave - Input floating/ Input pull-up / Input pull-down
     *             Hardware master/ NSS output enabled - Alternate function push-pull
     *             Software - Not used. Can be used as a GPIO
     */
    
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL5_Msk, 5 << GPIO_AFRL_AFSEL5_Pos);
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL6_Msk, 5 << GPIO_AFRL_AFSEL6_Pos);
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL7_Msk, 5 << GPIO_AFRL_AFSEL7_Pos);
    
    //SCK - PA5(Master - Alternate function push-pull):
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE5_Msk, 0b10 << GPIO_MODER_MODE5_Pos); //10: Alternate function mode
    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT5); //0: Output push-pull (reset state)
    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED5_Msk, 0b11 << GPIO_OSPEEDR_OSPEED5_Pos);//11: High speed
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD5_Msk, 0b00 << GPIO_PUPDR_PUPD5_Pos);//00: No pull-up, pull-down
    
    //MISO - PA6(Full duplex / master - Input floating / Input pull-up):
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE5_Msk, 0b10 << GPIO_MODER_MODE5_Pos); //10: Alternate function mode
    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED5_Msk, 0b11 << GPIO_OSPEEDR_OSPEED5_Pos); //11: High speed
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD5_Msk, 0b01 << GPIO_PUPDR_PUPD5_Pos); //01: pull-up
    
    //MOSI - PA7(Full duplex / master - Alternate function push-pull):
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE7_Msk, 0b10 << GPIO_MODER_MODE7_Pos); //10: Alternate function mode
    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT7); //0: Output push-pull (reset state)
    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED7_Msk, 0b11 << GPIO_OSPEEDR_OSPEED7_Pos); //11: High speed
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD7_Msk, 0b00 << GPIO_PUPDR_PUPD7_Pos); //00: No pull-up, pull-down 

     /*SPI control register 1 (SPI_CR1) (not used in I2S mode)(см. п.п. 25.5.1 стр 742)*/
    /*
     * Bits 5:3 BR[2:0]: Baud rate control
     * 000: fPCLK/2
     * 001: fPCLK/4
     * 010: fPCLK/8
     * 011: fPCLK/16
     * 100: fPCLK/32
     * 101: fPCLK/64
     * 110: fPCLK/128
     * 111: fPCLK/256
     * */
    MODIFY_REG(SPI1->CR1, SPI_CR1_BR, 0b000 << SPI_CR1_BR_Pos); //fPCLK/2. 96000000/2 = 48 MBits/s
    SET_BIT(SPI1->CR1, SPI_CR1_CPOL); //Полярность
    SET_BIT(SPI1->CR1, SPI_CR1_CPHA); //Фаза
    CLEAR_BIT(SPI1->CR1, SPI_CR1_DFF); //0: 8-bit data frame format is selected for transmission/reception
    CLEAR_BIT(SPI1->CR1, SPI_CR1_LSBFIRST); //0: MSB transmitted first
    SET_BIT(SPI1->CR1, SPI_CR1_SSM); //1: Software slave management enabled
    SET_BIT(SPI1->CR1, SPI_CR1_SSI); //1: Software slave management enabled
    SET_BIT(SPI1->CR1, SPI_CR1_MSTR); //1: Master configuration
    CLEAR_BIT(SPI1->CR1, SPI_CR1_BIDIMODE); //0: 2-line unidirectional data mode selected
    CLEAR_BIT(SPI1->CR1, SPI_CR1_RXONLY); //0: Full duplex (Transmit and receive)

   
    
    CLEAR_BIT(SPI1->CR1, SPI_CR1_CRCEN); //0: CRC calculation disabled
    CLEAR_BIT(SPI1->CR1, SPI_CR1_CRCNEXT); // 0: Data phase (no CRC phase) 
 


    /*SPI control register 2 (SPI_CR2) (см. п.п. 25.5.2 стр 744)*/
    CLEAR_BIT(SPI1->CR2, SPI_CR2_RXDMAEN); //0: Rx buffer DMA disabled
    CLEAR_BIT(SPI1->CR2, SPI_CR2_TXDMAEN); //0: Tx buffer DMA disabled
    CLEAR_BIT(SPI1->CR2, SPI_CR2_SSOE); //0: SS output is disabled in master mode and the cell can work in multimaster configuration
    CLEAR_BIT(SPI1->CR2, SPI_CR2_ERRIE); //0: Error interrupt is masked
    CLEAR_BIT(SPI1->CR2, SPI_CR2_FRF); //0: SPI Motorola mode
    CLEAR_BIT(SPI1->CR2, SPI_CR2_RXNEIE); //0: RXNE interrupt masked 
    CLEAR_BIT(SPI1->CR2, SPI_CR2_TXEIE); //0: TXE interrupt masked 

    /*SPI_I2S configuration register (SPI_I2SCFGR) (см. п.п. 25.5.8 стр 748)*/
    CLEAR_BIT(SPI1->I2SCFGR, SPI_I2SCFGR_I2SMOD); //т.к. на F103C6T6 нет I2S, его вырезали, а регистр оставили, нужно просто обнулить данный регистр. Тем самым включим режим SPI mode.

    SET_BIT(SPI1->CR1, SPI_CR1_SPE); //Включим SPI

}

/**
 **************************************************************************************************
 *  @breif Функция передачи данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, которые будем передавать.
 *  @param  Size_data - Размер, сколько байт будем передавать.
 *  @retval  Возвращает статус передачи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_SPI_Data_Transmit_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    //(см. Reference Manual стр. 712 Transmit-only procedure (BIDIMODE=0 RXONLY=0))
    if (!READ_BIT(SPI->SR, SPI_SR_BSY)) {
        //Проверим занятость шины
        SPI->DR = *(data); //Запишем первый элемент данных для отправки в регистр SPI_DR
        //(При этом очищается бит TXE)
        
        for (uint16_t i = 1; i < Size_data; i++) {
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->SR, SPI_SR_TXE)) {
                //Ждем, пока буфер на передачу не освободится
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            SPI->DR = *(data + i); //Запишем следующий элемент данных.
        }
        Timeout_counter_ms = Timeout_ms;
        while (!READ_BIT(SPI->SR, SPI_SR_TXE)) {
            //После записи последнего элемента данных в регистр SPI_DR,
            //подождем, пока TXE станет равным 1.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->SR, SPI_SR_BSY)) {
            //Затем подождем, пока BSY станет равным 0.
            //Это указывает на то, что передача последних данных завершена.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
    //Примечание:
    //После передачи двух элементов данных в режиме "transmit-only mode" в регистре SPI_SR устанавливается флаг OVR, так как принятые данные никогда не считываются.
}

/**
 **************************************************************************************************
 *  @breif Функция передачи данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, которые будем передавать.
 *  @param  Size_data - сколько 16 - битных данных хотим передать. Т.е. 1 = 2 байта.
 *  @retval  Возвращает статус передачи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_SPI_Data_Transmit_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    //(см. Reference Manual стр. 712 Transmit-only procedure (BIDIMODE=0 RXONLY=0))
    if (!READ_BIT(SPI->SR, SPI_SR_BSY)) {
        //Проверим занятость шины
        SPI->DR = *(data); //Запишем первый элемент данных для отправки в регистр SPI_DR
        //(При этом очищается бит TXE)
        
        for (uint16_t i = 1; i < Size_data; i++) {
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->SR, SPI_SR_TXE)) {
                //Ждем, пока буфер на передачу не освободится
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            SPI->DR = *(data + i); //Запишем следующий элемент данных.
        }
        Timeout_counter_ms = Timeout_ms;
        while (!READ_BIT(SPI->SR, SPI_SR_TXE)) {
            //После записи последнего элемента данных в регистр SPI_DR,
            //подождем, пока TXE станет равным 1.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->SR, SPI_SR_BSY)) {
            //Затем подождем, пока BSY станет равным 0.
            //Это указывает на то, что передача последних данных завершена.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
    //Примечание:
    //После передачи двух элементов данных в режиме "transmit-only mode" в регистре SPI_SR устанавливается флаг OVR, так как принятые данные никогда не считываются.
}

/**
 **************************************************************************************************
 *  @breif Функция приема данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, куда будем записывать принятые данные.
 *  @param  Size_data - Размер, сколько байт хотим принять.
 *  @retval  Возвращает статус приема. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_SPI_Data_Receive_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    if (!READ_BIT(SPI->SR, SPI_SR_BSY)) {
        //Проверим занятость шины
        
        if (READ_BIT(SPI->SR, SPI_SR_OVR) || READ_BIT(SPI->SR, SPI_SR_RXNE)) {
            //Т.к. мы можем принимать данные в любой момент, например после режима "transmit-only mode"
            //то следует проверить статусы OVR и RXNE. Если хотя бы один из них установлен, 
            //то сбросим их при помощи чтения регистра DR.
            SPI->DR;
        }
        
        //Начнем прием данных
        for (uint16_t i = 0; i < Size_data; i++) {
            SPI->DR = 0; //Запустим тактирование, чтоб считать 8 бит
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->SR, SPI_SR_RXNE)) {
                //Ждем, пока буфер на прием не заполнится
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            *(data + i) = SPI->DR; //Считываем данные
        }
        
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->SR, SPI_SR_BSY)) {
            //Затем подождем, пока BSY станет равным 0.
            //Это указывает на то, что прием последних данных завершен.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif Функция приема данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, куда будем записывать принятые данные.
 *  @param  Size_data - Размер, сколько 16 - битных данных хотим принять. Т.е. 1 = 2 байта.
 *  @retval  Возвращает статус приема. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_SPI_Data_Receive_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    if (!READ_BIT(SPI->SR, SPI_SR_BSY)) {
        //Проверим занятость шины
        
        if (READ_BIT(SPI->SR, SPI_SR_OVR) || READ_BIT(SPI->SR, SPI_SR_RXNE)) {
            //Т.к. мы можем принимать данные в любой момент, например после режима "transmit-only mode"
            //то следует проверить статусы OVR и RXNE. Если хотя бы один из них установлен, 
            //то сбросим их при помощи чтения регистра DR.
            SPI->DR;
        }
        
        //Начнем прием данных
        for (uint16_t i = 0; i < Size_data; i++) {
            SPI->DR = 0; //Запустим тактирование, чтоб считать 16 бит
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->SR, SPI_SR_RXNE)) {
                //Ждем, пока буфер на прием не заполнится
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            *(data + i) = SPI->DR; //Считываем данные
        }
        
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->SR, SPI_SR_BSY)) {
            //Затем подождем, пока BSY станет равным 0.
            //Это указывает на то, что прием последних данных завершен.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
}
