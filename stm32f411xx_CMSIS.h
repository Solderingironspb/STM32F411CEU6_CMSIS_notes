#include <main.h>
#include <stdbool.h>

void CMSIS_Debug_init(void);
void CMSIS_RCC_SystemClock_96MHz(void);
void CMSIS_SysTick_Timer_init(void);
void Delay_ms(uint32_t Milliseconds); //Функция задержки
void SysTick_Handler(void); //Прерывания от системного таймера
void CMSIS_PC13_OUTPUT_Push_Pull_init(void);
void CMSIS_Blink_PC13(uint32_t ms);
void CMSIS_SPI1_init(void); //Инициализация SPI1
bool CMSIS_SPI_Data_Transmit_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция отправки данных по SPI
bool CMSIS_SPI_Data_Transmit_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция отправки данных по SPI
bool CMSIS_SPI_Data_Receive_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция приема данных по SPI
bool CMSIS_SPI_Data_Receive_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция приема данных по SPI
bool CMSIS_SPI_Data_Transmit_fast(SPI_TypeDef* SPI, GPIO_TypeDef* GPIO, uint8_t NSS_pin, bool NSS_logic, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция передачи данных по SPI(быстрая. CS уже включен в нее)
bool CMSIS_SPI_Data_Receive_fast(SPI_TypeDef* SPI, GPIO_TypeDef* GPIO, uint8_t NSS_pin, bool NSS_logic, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция приема данных по SPI(быстрая. CS уже включен в нее)
