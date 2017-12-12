#ifndef __I2C_LIB_H
#define __I2C_LIB_H

#include "stm32f10x.h"

void InitI2C(void);
uint8_t I2C_ByteRead(uint8_t I2C_addr, uint8_t REG_Address);

void I2C_ByteWrite_Reg(uint8_t I2C_addr,uint8_t REG_Address,uint8_t REG_data);
void I2C_ByteWrite(uint8_t I2C_addr,uint8_t REG_data);


short ms5611ReadShort(uint8_t I2C_addr,unsigned char address);
unsigned long ms5611ReadLong(unsigned char I2C_Addr,unsigned char address);
void ms5611WriteByte(unsigned char I2C_Addr,unsigned char data);


/* HMC5883L */
/*
void  hmc5883lRead(unsigned char I2C_Addr,unsigned char reg_Addr,
                   unsigned char *pBuff,unsigned short numbyte);
*/
void hmc5883lWriteByte(unsigned char I2C_Addr,unsigned char data, unsigned char regAddr);

#endif



