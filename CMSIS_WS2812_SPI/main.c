#include "stm32f10x.h"

#define SYSCLOCK 40000000U
#define NUM_LED 36
uint8_t LED_Data[NUM_LED][4];
#define USE_BRIGHTNESS 1
int brightness = 1;

__IO uint32_t tmpreg;
__IO uint32_t SysTick_CNT = 0;
__IO uint8_t  tim2_count = 0;

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


void RCC_Init_HSI_PLL_5MHz(void) {
    RCC->CR |= RCC_CR_HSION; // Включаем внутренний генератор HSI
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Ждем, пока HSI стабилизируется

    // Настраиваем PLL
    RCC->CFGR &= ~RCC_CFGR_PLLMULL; // Очищаем биты множителя PLL
    RCC->CFGR |= RCC_CFGR_PLLMULL10; // Устанавливаем множитель PLL на 5 (8 МГц * 5 = 40 МГц)
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;  // Выбираем HSI как источник PLL

    // Включаем PLL
    RCC->CR |= RCC_CR_PLLON; // Включаем PLL
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Ждем, пока PLL стабилизируется

    // Настраиваем предделители
    RCC->CFGR &= ~RCC_CFGR_HPRE;  // Сбрасываем предделитель AHB (HCLK = SYSCLK)
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // Делим SYSCLK на 8 (40 МГц / 8 = 5 МГц)
    RCC->CFGR &= ~RCC_CFGR_PPRE1; // Сбрасываем предделитель APB1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV8; // Делим HCLK на 2 (6 МГц / 2 = 3 МГц)
    RCC->CFGR &= ~RCC_CFGR_PPRE2; // Сбрасываем предделитель APB2
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV8; // Делим HCLK на 2 (6 МГц / 2 = 3 МГц)

    // Переключаем систему на PLL
    RCC->CFGR &= ~RCC_CFGR_SW; // Сбрасываем биты выбора источника тактирования
    RCC->CFGR |= RCC_CFGR_SW_PLL; // Выбираем PLL как источник тактирования
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Ждем переключения
}

void SPI1_Init(void) {
    // Включение тактирования для SPI1 и GPIO (если SPI1 на портах A)
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;

    // PA7 (MOSI) - Alternate function push-pull
    GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
    GPIOA->CRL |= GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;

    // Настройка SPI1
    SPI1->CR1 = 0;  // Сброс регистра CR1

    // Настройка режима MODE 0 (CPOL = 0, CPHA = 1)
    SPI1->CR1 &= ~SPI_CR1_CPOL;  // CPOL = 0
    SPI1->CR1 |= SPI_CR1_CPHA;   // CPHA = 1

    // Настройка других параметров SPI
    SPI1->CR1 |= SPI_CR1_MSTR;  // Режим Master
    SPI1->CR1 |= 0x0345;  // Baud rate control (fPCLK/4)
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;  // Программное управление NSS
    SPI1->CR1 |= SPI_CR1_SPE;   // Включение SPI
}

void setLED (int led, int RED, int GREEN, int BLUE)
{
	LED_Data[led][0] = led;
	LED_Data[led][1] = GREEN;
	LED_Data[led][2] = RED;
	LED_Data[led][3] = BLUE;
}

void SPI_Transmit_CMSIS(SPI_TypeDef *SPIx, uint8_t *pData, uint16_t Size) {
    for (uint16_t i = 0; i < Size; i++) {
        // Ждем, пока SPI не будет готов к передаче (TXE = 1)
        while (!(SPIx->SR & SPI_SR_TXE));

        // Записываем данные в регистр данных SPI
        SPIx->DR = pData[i];
    }

    // Ждем, пока SPI завершит передачу (BSY = 0)
    while (SPIx->SR & SPI_SR_BSY);
}


void ws2812_spi (int GREEN, int RED, int BLUE)
{
#if USE_BRIGHTNESS
	if (brightness>100)brightness = 100;
	GREEN = GREEN*brightness/100;
	RED = RED*brightness/100;
	BLUE = BLUE*brightness/100;
#endif
	uint32_t color = GREEN<<16 | RED<<8 | BLUE;
	uint8_t sendData[24];
	uint16_t indx = 0;

	for (int i=23; i>=0; i--)
	{
		if (((color>>i)&0x01) == 1) 
sendData[indx++] = 0b110;  // store 1
		else 
sendData[indx++] = 0b100;  // store 0
	}

    // Передача данных через SPI (CMSIS)
    SPI_Transmit_CMSIS(SPI1, sendData, 24);
    for (int i=0; i<90; i++); // выставлена задержка между светиками в 200мкС
}

void WS2812_Send (void)
{
	for (volatile int i=0; i<NUM_LED; i++)
	{
		ws2812_spi(LED_Data[i][1], LED_Data[i][2], LED_Data[i][3]);
	}
	for (int i=0; i<180; i++);
}

int main(void)
{

  RCC_Init_HSI_PLL_5MHz();
  SysTick_Init();
  SPI1_Init();

	for (int i=0; i<NUM_LED; i++)
  {
	  setLED(i, 0, 0, 0);
  }
  WS2812_Send();

delay_ms (1000);
	while(1)
	{
	  for (int i=0; i<NUM_LED; i++)
	  {
		  setLED(i, 255, 0, 0);
	  }
	  WS2812_Send();
	 delay_ms (1000);

	  for (int i=0; i<NUM_LED; i++)
	  {
		  setLED(i, 0, 255, 0);
	  }
	  WS2812_Send();
  delay_ms (1000);


	  for (int i=0; i<NUM_LED; i++)
	  {
		  setLED(i, 0, 0, 255);
	  }
	  WS2812_Send();
	  delay_ms (1000);


	  for (int i=0; i<NUM_LED; i++)
	  {
		  setLED(i, 255, 255, 255);
	  }
	  WS2812_Send();
	  delay_ms (1000);
		}
}


void SysTick_Handler(void)
{
  if(SysTick_CNT > 0)  
     SysTick_CNT--;
}
