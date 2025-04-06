#include "i2c.h"

#define I2C_REQUEST_WRITE                     0x00
#define I2C_REQUEST_READ                      0x01
#define I2C_OWNADDRESS1_7BIT           0x00004000U
#define I2C_MODE_I2C                   0x00000000U

void I2C_Init(void)
{
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN; // GPIOB и AFIO

		// Ќастройка PB8 (SCL) и PB9 (SDA) в альтернативном режиме с открытым стоком
		AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;// –емаппинг I2C1 на PB8 и PB9
		GPIOB->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8 | GPIO_CRH_CNF9 | GPIO_CRH_MODE9); // —брос
		GPIOB->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_CNF9_1 |GPIO_CRH_CNF8_0 | GPIO_CRH_CNF9_0; // јльтернативна€ функци€ с открытым стоком
		GPIOB->CRH |= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE9_0 |GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_1; // –ежим вывода, 50 ћ√ц
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // I2C1

		I2C1->OAR2 &=~ I2C_OAR2_ENDUAL; //Disable  acknowledge on Own Address2 match address
		I2C1->CR1 &=~ I2C_CR1_ENGC; //Disable General Call
		I2C1->CR1 &=~ I2C_CR1_NOSTRETCH; //Enable Clock stretching
		
		I2C1->CR1 &=~ I2C_CR1_PE;// ќтключаем I2C перед настройкой
		I2C1->CR2 = 36; // ”станавливаем частоту APB1 (36 ћ√ц)
		I2C1->TRISE = 37; //// TRISE = 37 (1000 нс) - но APB1 у STM32F103 делитс€ на 2, поэтому частота APB1 здесь 36 ћ√ц. ≈сли стандартное врем€ нарастани€ дл€ I2C в стандартном режиме (100 к√ц) составл€ет 1000 нс, то подставл€ем в формулу: (36 * 1000) / 1000 + 1 = 36 + 1 = 37. ќтсюда значение 37.
		I2C1->CCR = 180; // CCR = 180 (SCL = 100 к√ц)

	  I2C1->CR1 = I2C_CR1_PE; // ¬ключаем I2C 
    I2C1->CR1 |= I2C_CR1_ACK; //TypeAcknowledge
		I2C1->OAR2 = I2C_OAR2_ADD2;
}
//----------------------------------------------------------

void I2C_SendByteByADDR(I2C_TypeDef * i2c, uint8_t c,uint8_t addr)
{
  //Disable Pos
  CLEAR_BIT(i2c->CR1, I2C_CR1_POS);
  MODIFY_REG(i2c->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
  SET_BIT(i2c->CR1, I2C_CR1_START);
  while (!READ_BIT(i2c->SR1, I2C_SR1_SB)){};            
  (void) i2c->SR1;
  //I2C_Write_Byte(addr);
  MODIFY_REG(i2c->DR, I2C_DR_DR, addr | I2C_REQUEST_WRITE);
  while (!READ_BIT(i2c->SR1, I2C_SR1_ADDR)){};
  (void) i2c->SR1;
  (void) i2c->SR2;
  //I2C_Write_Byte(c);
  MODIFY_REG(i2c->DR, I2C_DR_DR,c);
  while (!READ_BIT(i2c->SR1, I2C_SR1_TXE)){};
  //I2C_StopCondition();
  SET_BIT(i2c->CR1, I2C_CR1_STOP);
}
