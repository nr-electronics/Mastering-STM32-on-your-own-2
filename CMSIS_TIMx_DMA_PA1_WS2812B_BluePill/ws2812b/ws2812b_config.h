#ifndef __WS2812B_CONF_H__
#define __WS2812B_CONF_H__

#include <stdint.h>


#define WS2812B_NUM_LEDS        36 //Количество светодиодов в полоске/ленте

//Период следования бит в тиках таймера, должно быть около 1.25мкс
#define WS2812B_TIMER_AAR       0x005A

//Передача логического нуля около 0.4мкс
#define WS2812B_0_VAL           (WS2812B_TIMER_AAR / 3)

//Передача логической единицы около 0.85мкс
#define WS2812B_1_VAL           ((WS2812B_TIMER_AAR / 3) * 2)

//Сигнал RESET(RET), должен быть более 50мкс
#define WS2812B_TIMER_RET       (WS2812B_TIMER_AAR * 45)

//Разкомментировать, если нужно проиинвертировать выходной сигнал
//  #define WS2812B_OUTPUT_INVERSE
#endif
