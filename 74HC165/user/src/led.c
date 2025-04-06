#include "led.h"
uint8_t R1=0,R2=0,R3=0,R4=0;
uint16_t num_gl=0;
//==============================

#define SYSCLOCK 72000000U
#define TIM_EnableIT_UPDATE(TIMx) TIMx->DIER |= TIM_DIER_UIE
#define TIM_EnableCounter(TIMx) TIMx->CR1 |= TIM_CR1_CEN
#define TIM_DisableCounter(TIMx) TIMx->CR1 &=~ TIM_CR1_CEN

#define SA_SET    GPIOB->ODR |= (1 << 12)  // PB12
#define SA_RESET  GPIOB->ODR &= ~(1 << 12) // PB12

#define SB_SET    GPIOB->ODR |= (1 << 13)  // PB13
#define SB_RESET  GPIOB->ODR &= ~ (1 << 13)// PB13

#define SC_SET    GPIOB->ODR |= (1 << 14)  // PB14
#define SC_RESET  GPIOB->ODR &= ~ (1 << 14)// PB14

#define SD_SET    GPIOB->ODR |= (1 << 15)  // PB15
#define SD_RESET  GPIOB->ODR &= ~(1 << 15) // PB15

#define SE_SET    GPIOA->ODR |= (1 << 8)   // PA8
#define SE_RESET  GPIOA->ODR &= ~ (1 << 8) // PA8

#define SF_SET    GPIOA->ODR |= (1 << 9)   // PA9
#define SF_RESET  GPIOA->ODR &= ~(1 << 9) //  PA9

#define SG_SET    GPIOA->ODR |= (1 << 10)  // PA10
#define SG_RESET  GPIOA->ODR &= ~(1 << 10) // PB10

#define SDP_SET   GPIOA->ODR |= (1 << 11)  // PA11
#define SDP_RESET GPIOA->ODR &= ~ (1 << 11)// PB11

void segchar (uint8_t seg)
{
  switch(seg)
	{
		case 1:
			SA_RESET;SB_SET;SC_SET;SD_RESET;SE_RESET;SF_RESET;SG_RESET;
			break;
		case 2:
			SA_SET;SB_SET;SC_RESET;SD_SET;SE_SET;SF_RESET;SG_SET;
			break;
		case 3:
			SA_SET;SB_SET;SC_SET;SD_SET;SE_RESET;SF_RESET;SG_SET; 
			break;
		case 4:
			SA_RESET;SB_SET;SC_SET;SD_RESET;SE_RESET;SF_SET;SG_SET;
			break;
		case 5:
			SA_SET;SB_RESET;SC_SET;SD_SET;SE_RESET;SF_SET;SG_SET;
			break;
		case 6:
			SA_SET;SB_RESET;SC_SET;SD_SET;SE_SET;SF_SET;SG_SET;
			break;
		case 7:
			SA_SET;SB_SET;SC_SET;SD_RESET;SE_RESET;SF_RESET;SG_RESET; 
			break;
		case 8:
			SA_SET;SB_SET;SC_SET;SD_SET;SE_SET;SF_SET;SG_SET;
			break;
		case 9:
			SA_SET;SB_SET;SC_SET;SD_SET;SE_RESET;SF_SET;SG_SET;
			break;
		case 0:
			SA_SET;SB_SET;SC_SET;SD_SET;SE_SET;SF_SET;SG_RESET;
			break;
	}
}
//==============================
void ledprint(uint16_t number)
{
  num_gl=number;
	R1 = number%10;
	R2 = number%100/10;
	R3 = number%1000/100;
	R4 = number/1000;
}
