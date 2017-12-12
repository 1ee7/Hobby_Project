#include "i2c_lib.h"



/***************************************************************************************/
/*
 * 函数名：I2C_Mode_Config
 * 描述  ：I2C 工作模式配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void I2C_Mode_Config(void)
{
	 /* Initialize the I2C1 according to the I2C_InitStructure members */ 
	I2C_InitTypeDef I2C_InitStructure; 
	 
	  /* I2C 配置 */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C ; 
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; 
	//I2C_InitStructure.I2C_OwnAddress1 = SlaveAddress; 
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; 
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; 
	I2C_InitStructure.I2C_ClockSpeed = 50000; 


	/* I2C1 初始化 */
	I2C_Init(I2C1, &I2C_InitStructure);	   
	
	/* 使能 I2C1 */
	I2C_Cmd  (I2C1,ENABLE); 
	/*允许应答模式*/
	I2C_AcknowledgeConfig(I2C1, ENABLE);   
}


/*
 * 函数名：I2C_GPIO_Config
 * 描述  ：I2C1 I/O配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
		/* 使能与 I2C1 有关的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE ); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);  

	 /* PB6-I2C1_SCL、PB7-I2C1_SDA*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
}


/*
 * 函数名：I2C_Init
 * 描述  ：I2C 外设初始化
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void InitI2C(void)
{	   
 	I2C_GPIO_Config();
	I2C_Mode_Config();
}  




/*
 * I2C_ByteWrite_Reg
 * 描述  ：写一个字节到I2C设备寄存器中
 * 输入  ：REG_Address 接收数据的IIC设备寄存器的地址 
 *         REG_data 待写入的数据
 * 输出  ：无
 * 返回  ：无
 * 调用  ：内部调用
 */	

 
void I2C_ByteWrite_Reg(uint8_t I2C_addr,uint8_t REG_Address,uint8_t REG_data)
{

   I2C_GenerateSTART(I2C1,ENABLE);									  
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

   I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Transmitter);  
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

   I2C_SendData(I2C1,REG_Address);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

   I2C_SendData(I2C1,REG_data);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

   I2C_GenerateSTOP(I2C1,ENABLE);

}
/*
 * 函数名：I2C_ByteWrite
 * 描述  ：写一个字节到I2C设备中
 * 输入  ：REG_Address 接收数据的IIC设备寄存器的地址 
 *         REG_data 待写入的数据
 * 输出  ：无
 * 返回  ：无
 * 调用  ：内部调用
 */	
 void I2C_ByteWrite(uint8_t I2C_addr,uint8_t REG_data)
{
   I2C_GenerateSTART(I2C1,ENABLE);									  
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

   I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Transmitter);  
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

   //I2C_SendData(I2C1,REG_Address);
   //while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

   I2C_SendData(I2C1,REG_data);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

   I2C_GenerateSTOP(I2C1,ENABLE);

//   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

}
 

/*
 * 函数名：I2C_ByteRead
 * 描述  ：从IIC设备寄存器中读取一个字节
 * 输入  ：REG_Address 读取数据的寄存器的地址 
 * 输出  ：无
 * 返回  ：无
 * 调用  ：内部调用 
*/


uint8_t I2C_ByteRead(uint8_t I2C_addr, uint8_t REG_Address)
{
  uint8_t REG_data;

  while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));

  I2C_GenerateSTART(I2C1,ENABLE);//起始信号
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Transmitter);//发送设备地址+写信号
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//  
  I2C_Cmd(I2C1,ENABLE);
  I2C_SendData(I2C1,REG_Address);//发送存储单元地址，从0开始

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTART(I2C1,ENABLE);//起始信号

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Receiver);//发送设备地址+读信号

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  I2C_AcknowledgeConfig(I2C1,DISABLE);

  I2C_GenerateSTOP(I2C1,ENABLE);
  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));

  REG_data=I2C_ReceiveData(I2C1);//读出寄存器数据

  return REG_data;

}
/*   ms5611  */

short ms5611ReadShort(uint8_t I2C_addr,unsigned char address)
{
  unsigned short msb=0,lsb=0;

  I2C_AcknowledgeConfig(I2C1,ENABLE);
  I2C_GenerateSTART(I2C1,ENABLE);

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Transmitter);//发送设备地址+写信号

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C1,address);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Receiver);//发送设备地址+写信号
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
                             

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   msb=I2C_ReceiveData(I2C1);//读出寄存器数据

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   lsb=I2C_ReceiveData(I2C1);//读出寄存器数据

  I2C_GenerateSTOP(I2C1,ENABLE);
  I2C_AcknowledgeConfig(I2C1,DISABLE);

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   I2C_ReceiveData(I2C1);
  
   return (msb <<8)|lsb;

}


unsigned long ms5611ReadLong(unsigned char I2C_Addr,unsigned char address)
{
   unsigned long result=0,msb=0,lsb=0,xsb=0;

  I2C_AcknowledgeConfig(I2C1,ENABLE);
  I2C_GenerateSTART(I2C1,ENABLE);

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Transmitter);//发送设备地址+写信号

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C1,address);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Receiver);//发送设备地址+写信号
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   msb=I2C_ReceiveData(I2C1);//读出寄存器数据

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   lsb=I2C_ReceiveData(I2C1);//读出寄存器数据

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   xsb=I2C_ReceiveData(I2C1);//读出寄存器数据

  I2C_GenerateSTOP(I2C1,ENABLE);
  I2C_AcknowledgeConfig(I2C1,DISABLE);

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   I2C_ReceiveData(I2C1);
  
    result = (msb <<16)|(lsb<<8)|xsb;

	return result;
}

void ms5611WriteByte(unsigned char I2C_Addr,unsigned char data)
{
   I2C_GenerateSTART(I2C1,ENABLE);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

   I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Transmitter);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

   I2C_SendData(I2C1,data);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

   I2C_GenerateSTOP(I2C1,ENABLE);

   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));


}


/* HMC5883L */

void hmc5883lWriteByte(uint8_t I2C_Addr,uint8_t data, uint8_t regAddr)
{
   I2C_GenerateSTART(I2C1,ENABLE);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

   I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Transmitter);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

   I2C_SendData(I2C1,regAddr);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

   I2C_SendData(I2C1,data);
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

   I2C_GenerateSTOP(I2C1,ENABLE);
} 
/*
void  hmc5883lRead(uint8_t I2C_Addr,uint8_t reg_Addr,uint8_t *pBuff,uint16_t numbyte)
{
   uint8_t REG_data;

   while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));

   I2C_GenerateSTART(I2C1,ENABLE);//起始信号
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

   I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Transmitter);//发送设备地址+写信号
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//  

   I2C_Cmd(I2C1,ENABLE);
   I2C_SendData(I2C1,reg_Addr);//发送存储单元地址，从0开始
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

   I2C_GenerateSTART(I2C1,ENABLE);//起始信号

   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
   I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Receiver);//发送设备地址+读信号

   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
//	printf("\r\n:::::::::::::::\r\n");
   while(numbyte)
   {
      if(numbyte == 1)
	  {
	     I2C_AcknowledgeConfig(I2C1,DISABLE);
		 I2C_GenerateSTOP(I2C1,ENABLE);
	   }
      if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
	  {
	     *pBuff = I2C_ReceiveData(I2C1);//读出寄存器数据

	//	 printf("\r\n %x ",*pBuff);

		 pBuff++;

		 numbyte--;

	  }
	}
//	printf("\r\n:::::::::::::::\r\n");
//	I2C_AcknowlegetConfig(I2C1,ENABLE);

  I2C_GenerateSTOP(I2C1,ENABLE);
  I2C_AcknowledgeConfig(I2C1,DISABLE);
    
}
 */

