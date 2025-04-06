#ifndef I2C_USER_H_
#define I2C_USER_H_
//------------------------------------------------
#include "stm32f10x.h"
//------------------------------------------------
void I2C_SendByteByADDR(I2C_TypeDef * i2c, uint8_t c,uint8_t addr);
void I2C_Init(void);
//------------------------------------------------
#endif /* I2C_USER_H_ */
