#include "stm32f10x.h"
#include <stdio.h>

#define SYSCLOCK 72000000U

__IO uint32_t tmpreg;
__IO uint32_t SysTick_CNT = 0;
__IO uint8_t  tim2_count = 0;
//		uint32_t  captured_value;
volatile uint16_t pulse_count = 0;// ������� ��������� � ������
volatile uint32_t last_capture = 0; // ��������� �������� �������
volatile uint32_t time_between_pulses = 0; // ����� ����� ����������
volatile uint8_t packet_ready = 0; // ���� ���������� ������
volatile uint8_t measurement_done = 0; // ���� ���������� ���������

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

// ������������� GPIO ��� PB5 (TIM3_CH2)
void GPIO_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN ; // �������� ������������ GPIOB
		AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP;

    // ��������� PB5 ��� ���� (TIM3_CH2)
		GPIOB->CRL &= GPIO_CRL_MODE5_0;// ������� ��������� ��� PB5
		GPIOB->CRL &= GPIO_CRL_MODE5_1;
		GPIOB->CRL |= GPIO_CRL_CNF5_0;// PB5: ���� � ��������� ���������� (CNF = 01, MODE = 00)
		GPIOB->CRL &= GPIO_CRL_CNF5_1;

		// USART1 ���������:	
			// ��������� ������ �� ��������
		AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
		// ��������� PB7 (RX) ��� ���� � ���������
		GPIOB->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7); // ����� �����
		GPIOB->CRL |= GPIO_CRL_CNF7_0;                  // ���� � ���������
		GPIOB->ODR |= GPIO_ODR_ODR7;                    // ��������� �������� �����

		// ��������� PB6 (TX) ��� �������������� ������� (push-pull)
		GPIOB->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6); // ����� �����
		GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6;  // �������������� �������, 50 ���

			// ��������� USART1 ����������
		RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;	// USART1 Clock ON
		USART1->BRR 	= 0x1D4C;									// Baudrate for 9600 and 72Mhz-RCC
		USART1->CR1 	|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;	
			// USART1 ON, TX ON, RX ON

//		//NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
//		CLEAR_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG);
//		SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
}

// ������������� TIM3 ��� ������� ��������� �� ������ CH2
void TIM3_Init(void) {
    // ��������� ������������ TIM3 � GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN  | RCC_APB2ENR_AFIOEN;

    // ��������� ���������� TIM3: TIM3_REMAP[1:0] = 10 (��������� ���������)
    AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP; // ����� ����� ����������
    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_1; // ��������� ���������� ����������

    // ��������� PB5 ��� ���� � ��������� ����������
    GPIOB->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
    GPIOB->CRL |= GPIO_CRL_CNF5_1;

    // ��������� ���������� (8 ������)
    TIM3->CCMR1 &= ~TIM_CCMR1_IC2F; // ������� ����� IC2F
    TIM3->CCMR1 |= (0x5 << 12); // ��������� IC2F = 0x5 (8 ������)

    // ��������� TIM3 ��� ������� �� ������ 2
    TIM3->CCMR1 |= TIM_CCMR1_CC2S_0; // CC2 channel is configured as input, IC2 is mapped on TI2
    TIM3->CCER |= TIM_CCER_CC2E; // ��������� ������� �� ������ 2
    TIM3->DIER |= TIM_DIER_CC2IE; // ��������� ���������� �� ������� ������ 2

    TIM3->CR1 |= TIM_CR1_CEN; // ��������� �������
    // ��������� ���������� TIM3
    NVIC_EnableIRQ(TIM3_IRQn);
		//__enable_irq ();
}


void TIM2_Init(void) {
    // ��������� ������������ TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // ��������� TIM2
    TIM2->PSC = 7200 - 1; // ������������ (72 ��� / 7200 = 10 ���) 
    TIM2->ARR = 520 - 1; // ���������������� (10 ��� * 0.053 � = 530) 
    TIM2->DIER |= TIM_DIER_UIE; // ��������� ���������� �� ������������
    TIM2->CR1 |= TIM_CR1_CEN; // ��������� �������

    // ��������� ���������� TIM2
    NVIC_EnableIRQ(TIM2_IRQn);
}



// ����� ������ �� USART2
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
    // ��������� ������� ��������
    if (TIM3->SR & TIM_SR_CC2IF) {
		TIM3->SR &= ~TIM_SR_CC2IF; // ������� �����
        pulse_count++; // ���������� �������� ���������
    }
}

// ���������� ���������� TIM2 (���������� ���������)
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) { // �������� ����� ������������
        TIM2->SR &= ~TIM_SR_UIF; // ������� �����
        measurement_done = 1; // ��������� ����� ���������� ���������
        TIM2->CR1 &= ~TIM_CR1_CEN; // ��������� ������� TIM2
    }
}


// ���������� �������� Temp �� �������
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
  TIM3_Init();    // ������������� TIM3
  TIM2_Init();

    while (1) {
        if (measurement_done) { // ���� ��������� ���������
            // ���������� �������� Temp
            float temp = CalculateTemp(pulse_count);

            // �������� �������� Temp �� USART2
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "Temp: %.2f C\r\n", temp);
            USART1_SendString(buffer);

            // ����� �������� � �����
            pulse_count = 0;
            measurement_done = 0;

            // ���������� TIM2 ��� ������ ���������
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
