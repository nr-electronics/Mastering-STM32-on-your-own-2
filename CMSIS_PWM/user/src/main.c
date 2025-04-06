#include "stm32f10x.h"

#define SYSCLOCK 72000000U
#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)
#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_CC_EnableChannel(TIMx, Channels) SET_BIT(TIMx->CCER, Channels);

__IO uint32_t tmpreg;
__IO uint32_t SysTick_CNT = 0;
__IO uint8_t tim2_count = 0;

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

static void TIM2_Init(void)
{
  uint32_t tmpcr1, tmpcr2, tmpccer, tmpccmr2;
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
	AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP;
  tmpcr1 = READ_REG(TIM2->CR1);
  CLEAR_BIT(tmpcr1, TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD);
  WRITE_REG(TIM2->CR1, tmpcr1);
  //Set the auto-reload value
  WRITE_REG(TIM2->ARR, 65535);
  //Set the prescaler value
  WRITE_REG(TIM2->PSC, 0);
  //Generate an update event to reload the Prescaler
  //and the repetition counter value (if applicable) immediately
  SET_BIT(TIM2->EGR, TIM_EGR_UG);
  //Disable auto-reload
  CLEAR_BIT(TIM2->CR1, TIM_CR1_ARPE);
  //Set clock source internal
  CLEAR_BIT(TIM2->SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE);
  //CH3 AND CH4 Enable Preload
  SET_BIT(TIM2->CCMR2, TIM_CCMR2_OC4PE | TIM_CCMR2_OC3PE);
  //Disable the Channel 3 and 4: Reset the CC3E and CC4E Bits
  CLEAR_BIT(TIM2->CCER, TIM_CCER_CC4E | TIM_CCER_CC3E);
  //Get the TIM2 CCER register value
  tmpccer = READ_REG(TIM2->CCER);
  //Get the TIM2 CR2 register value
  tmpcr2 = READ_REG(TIM2->CR2);
  //Get the TIM2 CCMR2 register value
  tmpccmr2 = READ_REG(TIM2->CCMR2);
  //Reset Capture/Compare selection Bits
  CLEAR_BIT(tmpccmr2, TIM_CCMR2_CC4S | TIM_CCMR2_CC3S);
  //Select the Output Compare Mode
  MODIFY_REG(tmpccmr2, TIM_CCMR2_OC4M | TIM_CCMR2_OC3M, \
                       TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | \
                       TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
  //Set the Output Compare Polarity
  CLEAR_BIT(tmpccer, TIM_CCER_CC4P | TIM_CCER_CC3P);
  //Set the Output State
  CLEAR_BIT(tmpccer, TIM_CCER_CC4E | TIM_CCER_CC3E);
  //Write to TIM2 CR2
  WRITE_REG(TIM2->CR2, tmpcr2);
  //Write to TIM2 CCMR2
  WRITE_REG(TIM2->CCMR2, tmpccmr2);
  //Set the Capture Compare Registers value
  WRITE_REG(TIM2->CCR3, 0);
  WRITE_REG(TIM2->CCR4, 0);
  //Write to TIM2 CCER
  WRITE_REG(TIM2->CCER, tmpccer);
  //TIM2 OC Disable Fast
  CLEAR_BIT(TIM2->CCMR2, TIM_CCMR2_OC4FE | TIM_CCMR2_OC3FE);
  //Disable Master Mode Selection
  CLEAR_BIT(TIM2->CR2, TIM_CR2_MMS);
  //Disable Master/Slave mode
  CLEAR_BIT(TIM2->SMCR, TIM_SMCR_MSM);
  //Set GPIO
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);
  MODIFY_REG(GPIOB->CRH, GPIO_CRH_CNF11_0 | GPIO_CRH_CNF10_0 | GPIO_CRH_MODE11_0 | GPIO_CRH_MODE10_0,\
             GPIO_CRH_CNF11_1 | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE11_1 | GPIO_CRH_MODE10_1);
    
}
//----------------------------------------------------------
static void TIM3_Init(void)
{
  uint32_t tmpcr1, tmpcr2, tmpccer, tmpccmr1, tmpccmr2; //определение переменных
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
  tmpcr1 = READ_REG(TIM3->CR1);
  CLEAR_BIT(tmpcr1, TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD);
  WRITE_REG(TIM3->CR1, tmpcr1);
  //Set the auto-reload value
  WRITE_REG(TIM3->ARR, 65535);
  //Set the prescaler value
  WRITE_REG(TIM3->PSC, 0);
  //Generate an update event to reload the Prescaler
  //and the repetition counter value (if applicable) immediately
  SET_BIT(TIM3->EGR, TIM_EGR_UG);
  //Disable auto-reload
  CLEAR_BIT(TIM3->CR1, TIM_CR1_ARPE);
  //Set clock source internal
  CLEAR_BIT(TIM3->SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE);
  //CH1, CH2, CH3 AND CH4 Enable Preload
  SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2PE | TIM_CCMR1_OC1PE);
  SET_BIT(TIM3->CCMR2, TIM_CCMR2_OC4PE | TIM_CCMR2_OC3PE);
  //Disable the Channel 1, 2, 3 and 4: Reset the CC3E and CC4E Bits
  CLEAR_BIT(TIM3->CCER, TIM_CCER_CC4E | TIM_CCER_CC3E | \
                        TIM_CCER_CC2E | TIM_CCER_CC1E);
  //Get the TIM3 CCER register value
  tmpccer = READ_REG(TIM3->CCER);
  //Get the TIM3 CR2 register value
  tmpcr2 = READ_REG(TIM3->CR2);
  //Get the TIM3 CCMR1 register value
  tmpccmr1 = READ_REG(TIM3->CCMR1);
  //Get the TIM3 CCMR2 register value
  tmpccmr2 = READ_REG(TIM3->CCMR2);
  //Reset Capture/Compare selection Bits
  CLEAR_BIT(tmpccmr1, TIM_CCMR1_CC2S | TIM_CCMR1_CC1S);
  CLEAR_BIT(tmpccmr2, TIM_CCMR2_CC4S | TIM_CCMR2_CC3S);
  //Select the Output Compare Mode
  MODIFY_REG(tmpccmr1, TIM_CCMR1_OC2M | TIM_CCMR1_OC1M, \
                       TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | \
                       TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
  MODIFY_REG(tmpccmr2, TIM_CCMR2_OC4M | TIM_CCMR2_OC3M, \
                       TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | \
                       TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
  //Set the Output Compare Polarity
  CLEAR_BIT(tmpccer, TIM_CCER_CC4P | TIM_CCER_CC3P | TIM_CCER_CC2P | TIM_CCER_CC1P);
  //Set the Output State
  CLEAR_BIT(tmpccer, TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E);
  //Write to TIM2 CR2
  WRITE_REG(TIM3->CR2, tmpcr2);
  //Write to TIM2 CCMR1
  WRITE_REG(TIM3->CCMR1, tmpccmr1);
  //Write to TIM2 CCMR2
  WRITE_REG(TIM3->CCMR2, tmpccmr2);
  //Set the Capture Compare Registers value
  WRITE_REG(TIM3->CCR1, 0);
  WRITE_REG(TIM3->CCR2, 0);
  WRITE_REG(TIM3->CCR3, 0);
  WRITE_REG(TIM3->CCR4, 0);
  //Write to TIM2 CCER
  WRITE_REG(TIM3->CCER, tmpccer);
  //TIM2 OC Disable Fast
  CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC2FE | TIM_CCMR1_OC1FE );
  CLEAR_BIT(TIM3->CCMR2, TIM_CCMR2_OC4FE | TIM_CCMR2_OC3FE );
  //Disable Master Mode Selection
  CLEAR_BIT(TIM3->CR2, TIM_CR2_MMS);
  //Disable Master/Slave mode
  CLEAR_BIT(TIM3->SMCR, TIM_SMCR_MSM);
  //Set GPIO
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN);
  MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7_0 | GPIO_CRL_CNF6_0 | GPIO_CRL_MODE7_0 | GPIO_CRL_MODE6_0,\
             GPIO_CRL_CNF7_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_MODE6_1);
  MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF1_0 | GPIO_CRL_CNF0_0 | GPIO_CRL_MODE1_0 | GPIO_CRL_MODE0_0,\
             GPIO_CRL_CNF1_1 | GPIO_CRL_CNF0_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE0_1);
}

int main(void)
{
  uint32_t i;

  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
  //Delay after an RCC peripheral clock enabling
  delay(1);

  //NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
  CLEAR_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG);
  SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

  SetRCC_ClockTo72();
  SysTick_Init();

  TIM2_Init();
  TIM_CC_EnableChannel(TIM2, TIM_CCER_CC4E | TIM_CCER_CC3E);
  TIM3_Init();
  TIM_CC_EnableChannel(TIM3, TIM_CCER_CC3E ); 

  TIM_EnableCounter(TIM2);
  TIM_EnableCounter(TIM3);

  while(1)
	{
    for(i=0; i<393934; i++) 
    {
      if       (i<65536)              WRITE_REG(TIM2->CCR3, i); //PB10
      else if ((i>65535)&&(i<131072)) WRITE_REG(TIM2->CCR3,131071-i);

      else if((i>131071)&&(i<196608)) WRITE_REG(TIM2->CCR4,i-131072); //PB11
      else if ((i>196607)&&(i<262144))WRITE_REG(TIM2->CCR4,262143-i);

      else if((i>262143)&&(i<327680)) WRITE_REG(TIM3->CCR3,i-262144);//PB0
      else if ((i>327679)&&(i<393216))WRITE_REG(TIM3->CCR3,393215-i);

      delay(200);
    }
  }
}

void SysTick_Handler(void)
{
  if(SysTick_CNT > 0)  
     SysTick_CNT--;
}
