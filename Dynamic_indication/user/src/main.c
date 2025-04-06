#include "stm32f10x.h"

uint8_t tim2_count = 0;
uint8_t n_count=0;
void segchar (uint8_t seg);
void ledprint(uint16_t number);
uint8_t R1=0,R2=0,R3=0,R4=0;
uint16_t i,num_gl = 0;
void segchar (uint8_t seg);

#define SYSCLOCK 72000000U
#define TIM_EnableIT_UPDATE(TIMx) TIMx->DIER |= TIM_DIER_UIE
#define TIM_EnableCounter(TIMx) TIMx->CR1 |= TIM_CR1_CEN
#define TIM_DisableCounter(TIMx) TIMx->CR1 &=~ TIM_CR1_CEN

// Макросы для удобства работы с выводами
#define SEG1_PIN_RESET    GPIOB->ODR |=  (1 << 5)   // PB5
#define SEG1_PIN_SET 		  GPIOB->ODR &= ~(1 << 5)   // PB5

#define SEG2_PIN_RESET    GPIOB->ODR |=  (1 << 4)   // PB4
#define SEG2_PIN_SET      GPIOB->ODR &= ~(1 << 4)   // PB4

#define SEG3_PIN_RESET    GPIOA->ODR |=  (1 << 15)  // PA15
#define SEG3_PIN_SET      GPIOA->ODR &= ~(1 << 15)  // PA15

#define SEG4_PIN_RESET    GPIOA->ODR |=  (1 << 12)  // PA12
#define SEG4_PIN_SET      GPIOA->ODR &= ~(1 << 12)  // PA12

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



uint32_t SysTick_CNT = 0;
void delay(__IO uint32_t tck)
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
void RCC_DeInit(void)
{
		RCC->CR |= RCC_CR_HSION;
		while(READ_BIT(RCC->CR, RCC_CR_HSIRDY == RESET)) {}
		MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
		CLEAR_REG(RCC->CFGR);
		while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET) {}
		RCC->CR &=~ RCC_CR_PLLON;
		while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET) {}
		RCC->CR &=~ (RCC_CR_HSEON | RCC_CR_CSSON);
		while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET) {}
		RCC->CR &=~ RCC_CR_HSEBYP;
		//Reset all CSR flags
		RCC->CSR |= RCC_CSR_RMVF;
}

void SetSysClockTo72(void)
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


// Инициализация GPIO
void GPIO_Init(void) {
    // Включаем тактирование портов GPIOA и GPIOB
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |RCC_APB2ENR_AFIOEN;

    // Настройка выводов Seg1-Seg4 как выходы
    GPIOA->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12 | GPIO_CRH_CNF15 | GPIO_CRH_MODE15);
		GPIOA->CRH |= (GPIO_CRH_MODE12_1 | GPIO_CRH_MODE15_1);
		GPIOA->CRH &= ~ (GPIO_CRH_MODE12_0 | GPIO_CRH_MODE15_0 );

    GPIOB->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4 | GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
		GPIOB->CRL |= (GPIO_CRL_MODE4_1 | GPIO_CRL_MODE5_1);
		GPIOB->CRL &= ~ (GPIO_CRL_MODE4_0 | GPIO_CRL_MODE5_0);

    // Настройка выводов сегментов A-G и DP как выходы
    GPIOB->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12 | GPIO_CRH_CNF13 | GPIO_CRH_MODE13 |
                    GPIO_CRH_CNF14 | GPIO_CRH_MODE14 | GPIO_CRH_CNF15 | GPIO_CRH_MODE15);
    GPIOB->CRH |= (GPIO_CRH_MODE12_0 | GPIO_CRH_MODE13_0 | GPIO_CRH_MODE14_0 | GPIO_CRH_MODE15_0);

    GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8 | GPIO_CRH_CNF9 | GPIO_CRH_MODE9 |
                    GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
    GPIOA->CRH |= (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE11_0);
		
		//NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
		CLEAR_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG);
		SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
}

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

void ledprint(uint16_t number)
{
		num_gl=number;
		R1 = number%10;
		R2 = number%100/10;
		R3 = number%1000/100;
		R4 = number/1000;
}

void SysTick_Init(void)
{
		MODIFY_REG(SysTick->LOAD,SysTick_LOAD_RELOAD_Msk,SYSCLOCK / 1000 - 1);
		SysTick->VAL &=~ SysTick_VAL_CURRENT_Msk;
		SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}
//----------------------------------------------------------
void TIM2_Init(void)
{
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		NVIC_EnableIRQ(TIM2_IRQn);
		TIM2->PSC = 3599;
		TIM2->ARR = 50;
}

void SysTick_Handler(void)
{
		if(SysTick_CNT > 0)  SysTick_CNT--;
}

void TIM2_IRQHandler(void)
{
  if(READ_BIT(TIM2->SR, TIM_SR_UIF))
  {
    TIM2->SR &=~ TIM_SR_UIF;

    if(tim2_count==0)
    {
    SEG1_PIN_SET;
		SEG2_PIN_RESET;
		SEG3_PIN_RESET;
		SEG4_PIN_RESET;
     segchar(R1);
    }

    if(tim2_count==1)
    {
    SEG1_PIN_RESET;
		SEG2_PIN_SET;
		SEG3_PIN_RESET;
		SEG4_PIN_RESET;
     segchar(R2);
    }

    if(tim2_count==2)
    {
    SEG1_PIN_RESET;
		SEG2_PIN_RESET;
		SEG3_PIN_SET;
		SEG4_PIN_RESET;
     segchar(R3);
    }

    if(tim2_count==3)
    {
    SEG1_PIN_RESET;
		SEG2_PIN_RESET;
		SEG3_PIN_RESET;
		SEG4_PIN_SET;
      segchar(R4);
    }

    tim2_count++;
    if(tim2_count>3) tim2_count=0;
	}
}

int main(void) {
	RCC_DeInit();
	SetSysClockTo72();
  GPIO_Init();
  SysTick_Init();
  TIM2_Init();
  TIM_EnableIT_UPDATE(TIM2);
  TIM_EnableCounter(TIM2);

  while(1)
	{
    for(i=0; i<10000;i++)
    {
      ledprint(i);
      delay_ms(100);
    }
	}
}
