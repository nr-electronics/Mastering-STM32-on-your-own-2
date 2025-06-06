#ifndef __WS2812B_H__
#define __WS2812B_H__

#include <stdint.h>

//Инициализация настроек RGB-светодиода ws2812b
void ws2812b_init(void);

//Очистить буфер для светиков; Устанавливает всем светодиодам значения в 0;
//R=0, G=0, B=0;
void ws2812b_buff_clear(void);

//Установить RGB-светодиода номер pixn, PINx=0..WS2812B_NUM_LEDS-1
//r=0..255, g=0..255, b=0..255
//Возвращаемые значения
// 0 - выполнено успешно
// 1 - неверное значение PINx
int ws2812b_set(int pixn, uint8_t r, uint8_t g, uint8_t b);

//Загрузить подготовленный буфрер в ленту.
//Возвращает 1 если предыдущая операция обмена данными еще не завершена
int ws2812b_send(void);

//Возвращает 1 если предыдущая операция обмена данными с светодиодной лентой
//завершена успешно
int ws2812b_is_ready(void);


#endif

