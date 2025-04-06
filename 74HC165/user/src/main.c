#include "stm32f10x.h"
#include <stdio.h>

#define Sysclock 72000000U
uint8_t data,x = 0;
__IO uint16_t delay_count;


void SysTick_Handler(void) //1ms
{
  if (delay_count>0)
  {   delay_count--;  }  
}

void delay_ms(uint16_t delay_temp)
{ 
  delay_count=delay_temp;
  while (delay_count) {} 
}

// Макросы для управления выводами
#define CP_HIGH GPIOA->BSRR |= GPIO_BSRR_BS8 //PA8
#define CP_LOW  GPIOA->BSRR |= GPIO_BSRR_BR8

#define PL_HIGH GPIOA->BSRR |= GPIO_BSRR_BS9 //PA9
#define PL_LOW  GPIOA->BSRR |= GPIO_BSRR_BR9

#define Q7_HIGH GPIOA->BSRR |= GPIO_BSRR_BS10 //PA10
#define Q7_LOW  GPIOA->BSRR |= GPIO_BSRR_BR10

// Переменная для хранения состояния кнопок
    uint8_t button_state2 = 0;

// Инициализация GPIO
void GPIO_Init(void) {
    // Включение тактирования GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;

    // Настройка PA8 (DS) как выход!
		GPIOA->CRH &= ~GPIO_CRH_MODE8_0;//0: Выход, максимальная частота 2 MHz;
		GPIOA->CRH |=  GPIO_CRH_MODE8_1;//1: Выход, максимальная частота 2 MHz;
    GPIOA->CRH &= ~GPIO_CRH_CNF8_0;//00: General purpose output push-pull — выход в режиме Push-pull;                          
		GPIOA->CRH &= ~GPIO_CRH_CNF8_1;//00: General purpose output push-pull — выход в режиме Push-pull; 

    // Настройка PA9 (DS) как выход!
		GPIOA->CRH &= ~GPIO_CRH_MODE9_0;//0: Выход, максимальная частота 2 MHz;
		GPIOA->CRH |=  GPIO_CRH_MODE9_1;//1: Выход, максимальная частота 2 MHz;
    GPIOA->CRH &= ~GPIO_CRH_CNF9_0;//00: General purpose output push-pull — выход в режиме Push-pull;                          
		GPIOA->CRH &= ~GPIO_CRH_CNF9_1;//00: General purpose output push-pull — выход в режиме Push-pull; 

    // Настройка PA10 (DS) как вход!
		GPIOA->CRH &= ~GPIO_CRH_MODE10_0;//Вход (значение после сброса);
		GPIOA->CRH &= ~GPIO_CRH_MODE10_1;//Вход (значение после сброса);   
		GPIOA->CRH &= ~GPIO_CRH_CNF10_0;//10: Input with pull-up / pull-down — вход с подтяжкой к питанию или к земле;
		GPIOA->CRH |= GPIO_CRH_CNF10_1; //10: Input with pull-up / pull-down — вход с подтяжкой к питанию или к земле; 
		
    // Настройка USART1
			// настройка вывода на передачу
		GPIOA->CRL		&= ~GPIO_CRL_CNF2;				// 
		GPIOA->CRL		|= GPIO_CRL_CNF2_1;				// 
		GPIOA->CRL		|= GPIO_CRL_MODE2_0;
			// настройка вывода на прием
		GPIOA->CRL		&= ~GPIO_CRL_CNF3;				// 
		GPIOA->CRL		|= GPIO_CRL_CNF3_0;			// 
		GPIOA->CRL		&= ~GPIO_CRL_MODE3_1;
			// Настройка USART2 регистрами
		RCC->APB1ENR	|= RCC_APB1ENR_USART2EN;	// USART2 Clock ON
    USART2->BRR = 0xEA6;  // 9600 бод при тактовой частоте 36 МГц (APB1) 72Mhz-RCC	 	
		USART2->CR1 	|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;	


		//NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
		CLEAR_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG);
		SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
}

// Функция для чтения данных из 74HC165
    uint8_t Read_74HC165(void) {
    uint8_t button_state = 0;
		
    // Активируем загрузку данных в регистр (PL в низкий уровень)
    PL_LOW; // PL = 0
    PL_HIGH; // PL = 1

    // Чтение 8 бит данных
    for (int i = 0; i < 8; i++) 
		{
		    button_state |= ((GPIOA->IDR & GPIO_IDR_IDR10) ? 1 : 0) << i; // Чтение бита

        CP_HIGH; // CP = 1 (тактовый импульс)
        CP_LOW;  // CP = 0
    }
		button_state = ~button_state;
    return button_state;
}

// Отправка символа через UART
void UART_SendChar(char ch) {
    while (!(USART2->SR & USART_SR_TXE)); // Ждем, пока буфер TX не освободится
    USART2->DR = ch; // Отправляем символ
}

// Отправка строки через UART
void UART_SendString(const char *str) {
    while (*str) {
        UART_SendChar(*str++);
    }
}


int main(void) {
	//	char data = 0;
    GPIO_Init();
		SysTick_Config(SystemCoreClock/1000); //запуск систика для задержкек

  while (1) {
  uint8_t button_state = 0; 
	button_state = Read_74HC165();


				// Проверка состояния каждой кнопки
        for (int i = 0; i < 8; i++) {
            if (button_state & (1 << i)) {  //
                // Если кнопка нажата, отправляем информацию о кнопке через UART2 интерфейса USB
                char buffer[24];
                snprintf(buffer, sizeof(buffer), "Button %d pressed\r\n\n", i + 1);
                UART_SendString(buffer);
            }
        }

			delay_ms(100);
    }
}
