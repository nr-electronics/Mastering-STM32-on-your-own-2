#include "lcd.h"
#include "i2c.h"
//------------------------------------------------
char str1[100];
uint8_t buf[1]={0};
uint8_t portlcd;
//------------------------------------------------
void delay_ms(uint32_t ms);
//------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
	micros *=(SystemCoreClock / 1000000) / 9;
	while (micros--);
}
//------------------------------------------------
__STATIC_INLINE void DelayNano(__IO uint32_t nanos)
{
  nanos = nanos * (SystemCoreClock / 1000000) / 9000;
  while (nanos--);
}
//------------------------------------------------
void LCD_WriteByteI2CLCD(uint8_t bt)
{
  I2C_SendByteByADDR(I2C1, bt,0x4E);
}
//------------------------------------------------
void sendhalfbyte(uint8_t c)
{
  c<<=4;
  LCD_WriteByteI2CLCD(portlcd|c);
  LCD_WriteByteI2CLCD((portlcd|=0x04)|c);
  DelayNano(200);
  LCD_WriteByteI2CLCD((portlcd&=~0x04)|c);
}
//------------------------------------------------
void sendbyte(uint8_t c, uint8_t mode)
{
	if(mode==0) rs_reset();
	else rs_set();
	uint8_t hc=0;
	hc=c>>4;
	sendhalfbyte(hc);
  sendhalfbyte(c);
}
//------------------------------------------------
void LCD_Clear(void)
{
	sendbyte(0x01,0);
	delay_ms(2);
}
//------------------------------------------------
void LCD_SendChar(char ch)
{
	sendbyte(ch,1);
}
//------------------------------------------------
void LCD_String(char* st)
{
	uint8_t i=0;
	while(st[i]!=0)
	{
		sendbyte(st[i],1);
		i++;
	}
}
//------------------------------------------------
void LCD_SetPos(uint8_t x, uint8_t y)
{
	switch(y)
	{
		case 0:
			sendbyte(x|0x80,0);
			break;
		case 1:
			sendbyte((0x35+x)|0x80,0);
			break;
	}
}
//------------------------------------------------
void LCD_ini(void)
{
  delay_ms(50);
  LCD_WriteByteI2CLCD(0);
  setwrite();//запись
  delay_ms(100);
  sendhalfbyte(0x03);
  DelayMicro(4500);
  sendhalfbyte(0x03);
  DelayMicro(4500);
  sendhalfbyte(0x03);
  DelayMicro(200);
  sendhalfbyte(0x02);
  sendbyte(0x28,0);//режим 4 бит, 2 линии (для нашего большого дисплея это 4 линии, шрифт 5х8
  sendbyte(0x08,0);//дисплей пока выключаем
  delay_ms(1);
  sendbyte(0x01,0);// уберем мусор
  delay_ms(2);
  sendbyte(0x06,0);// пишем влево
  delay_ms(1);
  sendbyte(0x0C,0);//дисплей включаем (D=1), курсоры никакие не нужны
  sendbyte(0x02,0);//курсор на место
  delay_ms(2);
  setled();//подсветка
}

