#include "stm32f10x.h"
#include <stdio.h>

#define SYSCLOCK 72000000U

__IO uint32_t tmpreg;
__IO uint32_t SysTick_CNT = 0;
__IO uint8_t  tim2_count = 0;

char data = 0;
char temp_str[20];               // Буфер для строки температуры
volatile uint16_t adc_value = 0; // Значение АЦП
volatile uint32_t pulse_count = 0; // Счетчик импульсов

__forceinline void delay(__IO uint32_t tck)
{
  while(tck)
  {
    tck--;
  }  
}

void delay_ms(uint32_t ms)
{
  MODIFY_REG(SysTick->VAL,SysTick_VAL_CURRENT_Msk,SYSCLOCK / 1000 - 1);
  SysTick_CNT = ms;
  while(SysTick_CNT) {}
}

void SysTick_Init(void)
{
  MODIFY_REG(SysTick->LOAD,SysTick_LOAD_RELOAD_Msk,SYSCLOCK / 1000 - 1);
  CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
  SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}

void SetRCC_ClockTo72(void)
{
		RCC-> CR |= RCC_CR_HSEON;
		while (READ_BIT (RCC-> CR, RCC_CR_HSERDY == RESET)) {}
		FLASH-> ACR &= ~FLASH_ACR_PRFTBE;
		FLASH-> ACR |= FLASH_ACR_PRFTBE;
		FLASH-> ACR &= ~FLASH_ACR_LATENCY;
		FLASH-> ACR |= FLASH_ACR_LATENCY_2;
		RCC-> CFGR &= ~RCC_CFGR_HPRE;
		RCC-> CFGR |= RCC_CFGR_HPRE_DIV1;
		RCC-> CFGR &= ~RCC_CFGR_PPRE2;
		RCC-> CFGR |= RCC_CFGR_PPRE2_DIV1;
		RCC-> CFGR &= ~RCC_CFGR_PPRE1;
		RCC-> CFGR |= RCC_CFGR_PPRE1_DIV2;
		RCC-> CFGR &= (uint32_t) ((uint32_t) ~ (RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
		RCC-> CFGR |= (uint32_t) (RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
		RCC-> CR |= RCC_CR_PLLON; // Включаем ФАПЧ (PLL)
		while (READ_BIT (RCC-> CR, RCC_CR_PLLRDY)!= (RCC_CR_PLLRDY)) {}
		RCC-> CFGR &= ~RCC_CFGR_SW;
		RCC-> CFGR |= RCC_CFGR_SW_PLL;
		while (READ_BIT (RCC-> CFGR, RCC_CFGR_SWS)!= RCC_CFGR_SWS_PLL) {}
}

void GPIO_Init(void) {
		RCC->APB2ENR  |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN| RCC_APB2ENR_IOPCEN;
																																															 
		// USART1 настройка:	
			// настройка вывода на передачу
		AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
		// Настройка PB7 (RX) как вход с подтяжкой
		GPIOB->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7); // Сброс битов
		GPIOB->CRL |= GPIO_CRL_CNF7_0;                  // Вход с подтяжкой
		GPIOB->ODR |= GPIO_ODR_ODR7;                    // Включение подтяжки вверх

		// Настройка PB6 (TX) как альтернативная функция (push-pull)
		GPIOB->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6); // Сброс битов
		GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6;  // Альтернативная функция, 50 МГц
			
			// Настройка USART1 регистрами
		RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;	// USART1 Clock ON
		USART1->BRR 	= 0x1D4C;									// Baudrate for 9600 and 72Mhz-RCC
		USART1->CR1 	|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;	
			// USART1 ON, TX ON, RX ON


    // Настройка PA3 как аналоговый вход (ADC1_IN3)
    GPIOA->CRL &= ~(0xF << 12); // Очищаем настройки для PA3
    GPIOA->CRL |= (0x0 << 12);  // PA3: аналоговый вход (CNF = 00, MODE = 00)
}

void ADC1_Init(void) {
    // Включаем тактирование ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Настройка ADC1
    ADC1->SQR3 = 3;                     // Выбираем канал 3 (PA3)
    ADC1->SMPR2 |= ADC_SMPR2_SMP3_2;    // Устанавливаем время выборки (239.5 циклов)
    ADC1->CR2 |= ADC_CR2_ADON;          // Включаем ADC1
}

// Чтение значения АЦП
uint16_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_ADON;          // Запускаем преобразование
    while (!(ADC1->SR & ADC_SR_EOC));   // Ждем завершения преобразования
    return ADC1->DR;                    // Возвращаем результат
}

// Вывод строки на USART2
void USART1_SendString(char *str) {
    while (*str) {
        while ((USART1->SR & USART_SR_TXE) == 0); // Ждем, пока флаг TXE не установится
        USART1->DR = *str++;                      // Отправляем символ
    }
}


int main(void)
{
  //uint32_t i;

  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
  //Delay after an RCC peripheral clock enabling
  delay(1);
  SetRCC_ClockTo72();
  SysTick_Init();
  ADC1_Init();    // Инициализация ADC1

  GPIO_Init();    // Инициализация GPIO

	
   while (1) {

        // Чтение значения АЦП
        adc_value = ADC1_Read();

        // Преобразование значения АЦП в напряжение (в вольтах)
        float voltage = (adc_value * 3.3f) / 4096.0f;

        // Преобразование напряжения в температуру (LM35DZ: 10 мВ/°C)
        float temperature = voltage * 100.0f;

        // Форматирование строки температуры
        snprintf(temp_str, sizeof(temp_str), "Temp: %.2f C\r\n", temperature);

        // Вывод строки на USART1
        USART1_SendString(temp_str);

				delay_ms(300);
    }
}

void SysTick_Handler(void)
{
  if(SysTick_CNT > 0)  
     SysTick_CNT--;
}
