#include <stdint.h>
#include "stm32f10x.h"
#include "ws2812b.h"
#include "ws2812b_config.h"

#define WS2812B_OUTPUT_PAx      1
//Какой вывод будем использовать для вывода на ленту
/*
 Значения  Вывод STM32
 0         PA0
 1         PA1
 2         PA2
 3         PA3
*/

int main(void)
{
  ws2812b_init();
   
  while(1)
	{
	for(uint8_t i=0; i<WS2812B_NUM_LEDS; i++) //перебор для каждого из 16 светодиодов
			{
				ws2812b_set(i, 255, 255, 255);
			}
			ws2812b_send();
  }
}
