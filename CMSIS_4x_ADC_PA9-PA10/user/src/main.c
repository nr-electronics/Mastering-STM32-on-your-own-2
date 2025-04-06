#include "stm32f10x.h"
#include <stdio.h>

#define SYSCLOCK 72000000U

__IO uint32_t tmpreg;
__IO uint32_t SysTick_CNT = 0;
__IO uint8_t  tim2_count = 0;

// ����������
volatile uint16_t adc_values[4] = {0}; // �������� ��� ��� PA0, PA1, PA2, PA3
char voltage_str[65];                 // ����� ��� ������ ����������

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
		RCC-> CR |= RCC_CR_PLLON; // �������� ���� (PLL)
		while (READ_BIT (RCC-> CR, RCC_CR_PLLRDY)!= (RCC_CR_PLLRDY)) {}
		RCC-> CFGR &= ~RCC_CFGR_SW;
		RCC-> CFGR |= RCC_CFGR_SW_PLL;
		while (READ_BIT (RCC-> CFGR, RCC_CFGR_SWS)!= RCC_CFGR_SWS_PLL) {}
}


// ������������� GPIO ��� PB5 (TIM3_CH2)
void GPIO_Init(void) {
		RCC->APB2ENR  |= RCC_APB2ENR_IOPAEN;// ���� � �������� � ���������� ������������� �������������� �������
																																															 // ���� B � � ��������
		// USART1 ���������:	
			// ��������� ������ �� ��������
		GPIOA->CRH		&= ~GPIO_CRH_CNF9;				// Clear CNF bit 9
		GPIOA->CRH		|= GPIO_CRH_CNF9_1;				// Set CNF bit 9 to 10 - AFIO Push-Pull
		GPIOA->CRH		|= GPIO_CRH_MODE9_0;

			// ��������� ������ �� �����
		GPIOA->CRH		&= ~GPIO_CRH_CNF10;				// Clear CNF bit 9
		GPIOA->CRH		|= GPIO_CRH_CNF10_0;			// Set CNF bit 9 to 01 HiZ
		GPIOA->CRH		&= ~GPIO_CRH_MODE10;
			
			// ��������� USART1 ����������
		RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;	// USART1 Clock ON
		USART1->BRR 	= 0x1D4C;									// Baudrate for 9600 and 72Mhz-RCC
		USART1->CR1 	|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE ;	
			// USART1 ON, TX ON, RX ON, RXNEIE - ��������� ���������� �� ������ ������

    // ��������� PA0, PA1, PA2, PA3 ��� ���������� �����
    GPIOA->CRL &= ~(0xFFFF << 0); // ������� ��������� ��� PA0-PA3
    GPIOA->CRL |= (0x0 << 0) | (0x0 << 4) | (0x0 << 8) | (0x0 << 12); // ���������� ���� (CNF = 00, MODE = 00)

		//NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
		CLEAR_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG);
		SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
}

void ADC1_Init(void) {
    // �������� ������������ ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // ��������� ADC1
    ADC1->SQR1 = (3 << 20); // 4 ������ � ������������������ (L = 3)
    ADC1->SQR3 = (0 << 0) | (1 << 5) | (2 << 10) | (3 << 15); // ������ 0, 1, 2, 3 (PA0, PA1, PA2, PA3)
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP2_2 | ADC_SMPR2_SMP3_2; // ����� ������� (239.5 ������)
    ADC1->CR2 |= ADC_CR2_ADON; // �������� ADC1
}

// ������ �������� ��� ��� ���� �������
void ADC1_ReadAll(void) {
//    for (int i = 0; i < 4; i++) {
//    ADC1->CR2 |= ADC_CR2_ADON;          // ��������� ��������������
//    while (!(ADC1->SR & ADC_SR_EOC));   // ���� ���������� ��������������
//    adc_values[i] = ADC1->DR;                  // ���������� ���������
   for (int i = 0; i < 4; i++) {
        ADC1->SQR3 = (i << 0); // �������� ����� i (PA0, PA1, PA2, PA3)
        ADC1->CR2 |= ADC_CR2_SWSTART; // ��������� ��������������
        ADC1->CR2 |= ADC_CR2_ADON;          // ��������� ��������������
        while (!(ADC1->SR & ADC_SR_EOC)); // ���� ���������� ��������������
        adc_values[i] = ADC1->DR; // ��������� ���������
  } 
}

// ����� ������ �� USART2
void USART1_SendString(char *str) {
    while (*str) {
        while ((USART1->SR & USART_SR_TXE) == 0); // ����, ���� ���� TXE �� �����������
        USART1->DR = *str++;                      // ���������� ������
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
  ADC1_Init();    // ������������� ADC1
  GPIO_Init();    // ������������� GPIO

	
   while (1) {

        // ������ �������� ��� ��� ���� �������
        ADC1_ReadAll();

        // �������������� �������� ��� � ���������� (� �������)
        float voltage0 = (adc_values[0] * 3.3f) / 4096.0f;
        float voltage1 = (adc_values[1] * 3.3f) / 4096.0f;
        float voltage2 = (adc_values[2] * 3.3f) / 4096.0f;
        float voltage3 = (adc_values[3] * 3.3f) / 4096.0f;

        // �������������� ������ ����������
        snprintf(voltage_str, sizeof(voltage_str), "PA0: %.2f V\r\nPA1: %.2f V\r\nPA2: %.2f V\r\nPA3: %.2f V\r\n\n__________\r\n\n",
                 voltage0, voltage1, voltage2, voltage3);

        // ����� ������ �� USART1
        USART1_SendString(voltage_str);

				delay_ms(500);
    }
}

void SysTick_Handler(void)
{
  if(SysTick_CNT > 0)  
     SysTick_CNT--;
}
