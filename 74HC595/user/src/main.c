#include "stm32f10x.h"

#define Sysclock 72000000U
uint8_t data = 0;
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

// ������� ��� ���������� ��������
#define DATA_HIGH   GPIOA->BSRR  |= GPIO_BSRR_BS12 // PA12 (Data)
#define DATA_LOW    GPIOA->BSRR  |= GPIO_BSRR_BR12

#define CLOCK_HIGH  GPIOB->BSRR |=  GPIO_BSRR_BS4 // PB4 (Clock)
#define CLOCK_LOW   GPIOB->BSRR |=  GPIO_BSRR_BR4

#define LATCH_HIGH GPIOA->BSRR |= GPIO_BSRR_BS15 //PA15
#define LATCH_LOW  GPIOA->BSRR |= GPIO_BSRR_BR15

// ������� ��� �������� �������� ������ (LSBFIRST)
void ShiftOut_LSBFIRST(uint8_t data) {
		
    for (int i = 0; i < 8; i++) {
        // ������������� �������� ���� �� ������ DATA
     //   if (data & (1 << i)) { // ��������� ������� ��� (LSB) - ������ ������� ������ ��������
				if (data & (1 << (7 - i))) {
            DATA_HIGH;
        } else {
            DATA_LOW;
        }
        // ���������� ������� �� CLK (SH_CP)
        CLOCK_HIGH;
        CLOCK_LOW;
    }
    // ���������� ������� �� LATCH (ST_CP)
    LATCH_HIGH;
		LATCH_LOW;
}

// ������������� GPIO
void GPIO_Init(void) {
    // ��������� ������������ GPIOA � GPIOB
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |RCC_APB2ENR_AFIOEN;

    // ��������� PA12 (DS) ��� �����
		GPIOA->CRH &= ~GPIO_CRH_MODE12_0;//0: �����, ������������ ������� 2 MHz;
		GPIOA->CRH |=  GPIO_CRH_MODE12_1;//1: �����, ������������ ������� 2 MHz;
    GPIOA->CRH &= ~GPIO_CRH_CNF12_0;//00: General purpose output push-pull � ����� � ������ Push-pull;                          
		GPIOA->CRH &= ~GPIO_CRH_CNF12_1;//00: General purpose output push-pull � ����� � ������ Push-pull; 

    // ��������� PA15 (LATCH) ��� �����
		GPIOA->CRH &= ~GPIO_CRH_MODE15_0;//0: �����, ������������ ������� 2 MHz;
		GPIOA->CRH |=  GPIO_CRH_MODE15_1;//1: �����, ������������ ������� 2 MHz;
    GPIOA->CRH &= ~GPIO_CRH_CNF15_0;//00: General purpose output push-pull � ����� � ������ Push-pull;                          
		GPIOA->CRH &= ~GPIO_CRH_CNF15_1;//00: General purpose output push-pull � ����� � ������ Push-pull; 

    // ��������� PB4 (CLK) ��� �����
		GPIOB->CRL &= ~GPIO_CRL_MODE4_0;//0: �����, ������������ ������� 2 MHz;
		GPIOB->CRL |=  GPIO_CRL_MODE4_1;//1: �����, ������������ ������� 2 MHz;
		GPIOB->CRL &= ~GPIO_CRL_CNF4_0;//00: General purpose output push-pull � ����� � ������ Push-pull;                          
		GPIOB->CRL &= ~GPIO_CRL_CNF4_1;//00: General purpose output push-pull � ����� � ������ Push-pull;
		
		//NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
		CLEAR_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG);
		SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
}

int main(void) {
    GPIO_Init();
		SysTick_Config(SystemCoreClock/1000); //������ ������� ��� ���������

    // �������� ���� 0x30 (B00110000) � ������ LSBFIRST
    uint16_t i = 0;

    while (1) {
        ShiftOut_LSBFIRST(i++);
				delay_ms(100);
    }
}
