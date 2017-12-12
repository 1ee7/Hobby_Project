#include "ms5611.h"
#include <math.h>

unsigned short ms5611_C1,ms5611_C2,ms5611_C3,ms5611_C4,ms5611_C5,ms5611_C6;

 /*
 short ms5611ReadShort(unsigned char address)
 {
   unsigned short msb = 0;
   unsigned short lsb = 0;

  msb=I2C_ByteRead(MS5611_I2C_Addr,address);
  lsb=I2C_ByteRead(MS5611_I2C_Addr,address+1);
  return (msb<<8)+lsb;   //合成数据

 }

 unsigned long ms5611ReadLong(unsigned char address)
{
	unsigned long result=0;
	unsigned long msb=0,lsb=0,xsb=0;

	msb = I2C_ByteRead(MS5611_I2C_Addr,address);
    lsb = I2C_ByteRead(MS5611_I2C_Addr,address+1);
	xsb = I2C_ByteRead(MS5611_I2C_Addr,address+2);
    result =  (msb<<16) | (lsb<<8) | xsb;

   return result;
}
*/
  



void MS5611_Init(void) 
{
  
   I2C_ByteWrite(MS5611_I2C_Addr,0x1E);
  // ms5611WriteByte(MS5611_I2C_Addr,0x1E);
   delay_ms(10);

   ms5611_C1 = ms5611ReadShort(MS5611_I2C_Addr,0xA2);
   ms5611_C2 = ms5611ReadShort(MS5611_I2C_Addr,0xA4);
   ms5611_C3 = ms5611ReadShort(MS5611_I2C_Addr,0xA6);
   ms5611_C4 = ms5611ReadShort(MS5611_I2C_Addr,0xA8);
   ms5611_C5 = ms5611ReadShort(MS5611_I2C_Addr,0xAA);
   ms5611_C6 = ms5611ReadShort(MS5611_I2C_Addr,0xAC);
  
   /*
   ms5611_C1 = ms5611ReadShort(0xA2);
   ms5611_C2 = ms5611ReadShort(0xA4);
   ms5611_C3 = ms5611ReadShort(0xA6);
   ms5611_C4 = ms5611ReadShort(0xA8);
   ms5611_C5 = ms5611ReadShort(0xAA);
   ms5611_C6 = ms5611ReadShort(0xAC);
  */
  
}

void MS5611_Convert(long *temperature,long *pressure)
{
   unsigned long D1,D2;
   long dT,TEMP;
   long long OFF,SENS,OFF2,SENS2,T2;

  // printf("the C1:%d,C2:%d,C3:%d,C4:%d,C5:%d,C6:%d \r\n",
   //  ms5611_C1,ms5611_C2,ms5611_C3,ms5611_C4,ms5611_C5,ms5611_C6);

 // 进行气压转化
   I2C_ByteWrite(MS5611_I2C_Addr,0x40+MS5611_OSR*2);
  // ms5611WriteByte(MS5611_I2C_Addr,0x40+MS5611_OSR*2);

   delay_ms(1+MS5611_OSR*2);
   //Read Pressure data
  // D1 = ms5611ReadLong(0x00);
   D1 = ms5611ReadLong(MS5611_I2C_Addr,0x00);

   //start temperature conversion （温度转化）
   // I2C_ByteWrite(MS5611_I2C_Addr,0x50+MS5611_OSR*2);
   ms5611WriteByte(MS5611_I2C_Addr,0x50+MS5611_OSR*2);
   delay_ms(1+MS5611_OSR*2);

   //Read Temperature data
   // D2 = ms5611ReadLong(0x00);
    D2 = ms5611ReadLong(MS5611_I2C_Addr,0x00);



   dT = D2-(ms5611_C5<<8);
   TEMP = 2000 + (( (long long)dT*(long long)ms5611_C6)>>23);
   OFF = ( (long long)ms5611_C2 <<16)+(((long long)ms5611_C4*(long long)dT)>>7);
   SENS = ((long long)ms5611_C1 <<15)+(((long long)ms5611_C3*(long long)dT)>>8);


   if(TEMP >= 2000)
   {
   		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
   }
  else if(TEMP < 2000)
  {
    T2 = (((long long )dT*(long long)dT) >>31);
	OFF2 = 5*(((long long)TEMP - 2000)*((long long)TEMP - 2000))>>1;
	SENS2 = 5*(((long long)TEMP -2000)*((long long)TEMP - 2000))>>2;

	if(TEMP < -1500)
	{
	  OFF2 = OFF2 + 7*(((long long)TEMP + 1500)*((long long)TEMP +1500));
	  SENS2 = SENS2 + ((11*(((long long)TEMP + 1500)*(( long long)TEMP + 1500)))>>1);
	}
  }


//	printf(" Press : %ld,  Temp : %ld , T2:%d,OFF2:%d,SENS2:%d \r\n",D1,D2,T2,OFF2,SENS2);

   TEMP=TEMP-T2;
   OFF=OFF-OFF2;
   SENS=SENS-SENS2;	     

  *pressure = (unsigned long)(((((D1*SENS)>>21)-OFF))>>15);
  *temperature = (long)TEMP;

}
