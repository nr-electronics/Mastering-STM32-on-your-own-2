#include "stm32f10x.h"
#include <stdio.h>

#define SYSCLOCK 72000000U

__IO uint32_t tmpreg;
__IO uint32_t SysTick_CNT = 0;
__IO uint8_t  tim2_count = 0;
//		uint32_t  captured_value;
volatile uint16_t pulse_count = 0;// Счетчик импульсов в пакете
volatile uint32_t last_capture = 0; // Последнее значение захвата
volatile uint32_t time_between_pulses = 0; // Время между импульсами
volatile uint8_t packet_ready = 0; // Флаг готовности пакета
volatile uint8_t measurement_done = 0; // Флаг завершения измерения

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

// Инициализация GPIO для PB5 (TIM3_CH2)
void GPIO_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN ; // Включаем тактирование GPIOB
		AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP;

    // Настройка PB5 как вход (TIM3_CH2)
		GPIOB->CRL &= GPIO_CRL_MODE5_0;// Очищаем настройки для PB5
		GPIOB->CRL &= GPIO_CRL_MODE5_1;
		GPIOB->CRL |= GPIO_CRL_CNF5_0;// PB5: вход с плавающим состоянием (CNF = 01, MODE = 00)
		GPIOB->CRL &= GPIO_CRL_CNF5_1;

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

//		//NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
//		CLEAR_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG);
//		SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
}

// Инициализация TIM3 для захвата импульсов на канале CH2
void TIM3_Init(void) {
    // Включение тактирования TIM3 и GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN  | RCC_APB2ENR_AFIOEN;

    // Настройка ремаппинга TIM3: TIM3_REMAP[1:0] = 10 (частичный ремаппинг)
    AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP; // Сброс битов ремаппинга
    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_1; // Установка частичного ремаппинга

    // Настройка PB5 как вход с плавающим состоянием
    GPIOB->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
    GPIOB->CRL |= GPIO_CRL_CNF5_1;

    // Настройка фильтрации (8 тактов)
    TIM3->CCMR1 &= ~TIM_CCMR1_IC2F; // Очистка битов IC2F
    TIM3->CCMR1 |= (0x5 << 12); // Установка IC2F = 0x5 (8 тактов)

    // Настройка TIM3 для захвата по каналу 2
    TIM3->CCMR1 |= TIM_CCMR1_CC2S_0; // CC2 channel is configured as input, IC2 is mapped on TI2
    TIM3->CCER |= TIM_CCER_CC2E; // Включение захвата на канале 2
    TIM3->DIER |= TIM_DIER_CC2IE; // Включение прерывания по захвату канала 2

    TIM3->CR1 |= TIM_CR1_CEN; // Включение таймера
    // Настройка прерывания TIM3
    NVIC_EnableIRQ(TIM3_IRQn);
		//__enable_irq ();
}


void TIM2_Init(void) {
    // Включение тактирования TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Настройка TIM2
    TIM2->PSC = 7200 - 1; // Предделитель (72 МГц / 7200 = 10 кГц) 
    TIM2->ARR = 520 - 1; // Автоперезагрузка (10 кГц * 0.053 с = 530) 
    TIM2->DIER |= TIM_DIER_UIE; // Включение прерывания по переполнению
    TIM2->CR1 |= TIM_CR1_CEN; // Включение таймера

    // Настройка прерывания TIM2
    NVIC_EnableIRQ(TIM2_IRQn);
}



// Вывод строки на USART2
void USART1_SendChar(char ch) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = ch;
}

void USART1_SendString(char *str) {
    while (*str) {
        USART1_SendChar(*str++);
    }
}


void TIM3_IRQHandler() {
    // Обработка захвата импульса
    if (TIM3->SR & TIM_SR_CC2IF) {
		TIM3->SR &= ~TIM_SR_CC2IF; // Очистка флага
        pulse_count++; // Увеличение счетчика импульсов
    }
}

// Обработчик прерывания TIM2 (завершение измерения)
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) { // Проверка флага переполнения
        TIM2->SR &= ~TIM_SR_UIF; // Очистка флага
        measurement_done = 1; // Установка флага завершения измерения
        TIM2->CR1 &= ~TIM_CR1_CEN; // Остановка таймера TIM2
    }
}


// Вычисление значения Temp по формуле
float CalculateTemp(uint32_t pulses) {
    return (pulses * 0.0625f) - 50.0625f;
}

int main(void)
{
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
  //Delay after an RCC peripheral clock enabling
  delay(1);

  GPIO_Init();
  SysTick_Init();
  TIM3_Init();    // Инициализация TIM3
  TIM2_Init();

    while (1) {
        if (measurement_done) { // Если измерение завершено
            // Вычисление значения Temp
            float temp = CalculateTemp(pulse_count);

            // Отправка значения Temp на USART2
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "Temp: %.2f C\r\n", temp);
            USART1_SendString(buffer);

            // Сброс счетчика и флага
            pulse_count = 0;
            measurement_done = 0;

            // Перезапуск TIM2 для нового измерения
            TIM2->CNT = 0;
            TIM2->CR1 |= TIM_CR1_CEN;
					}
    }
}

void SysTick_Handler(void)
{
  if(SysTick_CNT > 0)  
     SysTick_CNT--;
}
