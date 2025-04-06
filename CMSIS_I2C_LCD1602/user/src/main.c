#include "stm32f10x.h"
#include "lcd.h"
#include "i2c.h"
#include <stdio.h>

#define Sysclock 72000000U

uint32_t SysTick_CNT = 0;

void delay_ms(uint32_t ms)
{
		MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, Sysclock / 1000 - 1);
		SysTick_CNT = ms;
		while(SysTick_CNT) {}
}

void SysTick_Init(void)
{
		MODIFY_REG(SysTick->LOAD,SysTick_LOAD_RELOAD_Msk,Sysclock / 1000 - 1);
		SysTick->VAL &= SysTick_VAL_CURRENT_Msk;
		SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}
//----------------------------------------------------------
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
//----------------------------------------------------------
int main(void)
{
		uint16_t i;
		char str1[10];
		SetRCC_ClockTo72();
		SysTick_Init();
		I2C_Init();
		LCD_ini();
		LCD_String("Hi!NR.electronics");
		LCD_SetPos(5,1);
		LCD_String("String 2");
		delay_ms(2000);
		LCD_SetPos(1,1);
		LCD_String("            ");


  while(1)
	{
    sprintf(str1,"%5d",i++);
    LCD_SetPos(0,1);//Позиция 0 и строка 1
    LCD_String(str1);//Вывод строки
    delay_ms(1000);
  }
}
//----------------------------------------------------------
void SysTick_Handler(void)
{
		if(SysTick_CNT > 0)  
       SysTick_CNT--;
}
//----------------------------------------------------------
