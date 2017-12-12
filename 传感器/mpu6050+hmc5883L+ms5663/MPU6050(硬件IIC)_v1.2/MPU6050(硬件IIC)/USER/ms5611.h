#ifndef __MS5611_H
#define	__MS5611_H
#include "i2c_lib.h"
#include "delay.h"


/* MS5611 Adrress ---------------------------------------- */
#define MS5611_I2C_Addr 0xEE  //0xEE
#define MS5611_OSR 4 //0..4



void MS5611_Init(void);
void MS5611_Convert(long *temperature,long *pressure);


#endif



