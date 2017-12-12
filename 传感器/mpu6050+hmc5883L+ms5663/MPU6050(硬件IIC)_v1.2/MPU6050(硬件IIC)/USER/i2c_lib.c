#include "i2c_lib.h"



/***************************************************************************************/
/*
 * ��������I2C_Mode_Config
 * ����  ��I2C ����ģʽ����
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void I2C_Mode_Config(void)
{
	 /* Initialize the I2C1 according to the I2C_InitStructure members */ 
	I2C_InitTypeDef I2C_InitStructure; 
	 
	  /* I2C ���� */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C ; 
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; 
	//I2C_InitStructure.I2C_OwnAddress1 = SlaveAddress; 
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; 
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; 
	I2C_InitStructure.I2C_ClockSpeed = 50000; 


	/* I2C1 ��ʼ�� */
	I2C_Init(I2C1, &I2C_InitStructure);	   
	
	/* ʹ�� I2C1 */
	I2C_Cmd  (I2C1,ENABLE); 
	/*����Ӧ��ģʽ*/
	I2C_AcknowledgeConfig(I2C1, ENABLE);   
}


/*
 * ��������I2C_GPIO_Config
 * ����  ��I2C1 I/O����
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
		/* ʹ���� I2C1 �йص�ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE ); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);  

	 /* PB6-I2C1_SCL��PB7-I2C1_SDA*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
}


/*
 * ��������I2C_Init
 * ����  ��I2C �����ʼ��
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void InitI2C(void)
{	   
 	I2C_GPIO_Config();
	I2C_Mode_Config();
}  




/*
 * I2C_ByteWrite_Reg
 * ����  ��дһ���ֽڵ�I2C�豸�Ĵ�����
 * ����  ��REG_Address �������ݵ�IIC�豸�Ĵ����ĵ�ַ 
 *         REG_data ��д�������
 * ���  ����
 * ����  ����
 * ����  ���ڲ�����
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
 * ��������I2C_ByteWrite
 * ����  ��дһ���ֽڵ�I2C�豸��
 * ����  ��REG_Address �������ݵ�IIC�豸�Ĵ����ĵ�ַ 
 *         REG_data ��д�������
 * ���  ����
 * ����  ����
 * ����  ���ڲ�����
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
 * ��������I2C_ByteRead
 * ����  ����IIC�豸�Ĵ����ж�ȡһ���ֽ�
 * ����  ��REG_Address ��ȡ���ݵļĴ����ĵ�ַ 
 * ���  ����
 * ����  ����
 * ����  ���ڲ����� 
*/


uint8_t I2C_ByteRead(uint8_t I2C_addr, uint8_t REG_Address)
{
  uint8_t REG_data;

  while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));

  I2C_GenerateSTART(I2C1,ENABLE);//��ʼ�ź�
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Transmitter);//�����豸��ַ+д�ź�
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//  
  I2C_Cmd(I2C1,ENABLE);
  I2C_SendData(I2C1,REG_Address);//���ʹ洢��Ԫ��ַ����0��ʼ

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTART(I2C1,ENABLE);//��ʼ�ź�

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Receiver);//�����豸��ַ+���ź�

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  I2C_AcknowledgeConfig(I2C1,DISABLE);

  I2C_GenerateSTOP(I2C1,ENABLE);
  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));

  REG_data=I2C_ReceiveData(I2C1);//�����Ĵ�������

  return REG_data;

}
/*   ms5611  */

short ms5611ReadShort(uint8_t I2C_addr,unsigned char address)
{
  unsigned short msb=0,lsb=0;

  I2C_AcknowledgeConfig(I2C1,ENABLE);
  I2C_GenerateSTART(I2C1,ENABLE);

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Transmitter);//�����豸��ַ+д�ź�

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C1,address);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1,I2C_addr,I2C_Direction_Receiver);//�����豸��ַ+д�ź�
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
                             

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   msb=I2C_ReceiveData(I2C1);//�����Ĵ�������

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   lsb=I2C_ReceiveData(I2C1);//�����Ĵ�������

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
  I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Transmitter);//�����豸��ַ+д�ź�

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C1,address);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Receiver);//�����豸��ַ+д�ź�
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   msb=I2C_ReceiveData(I2C1);//�����Ĵ�������

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   lsb=I2C_ReceiveData(I2C1);//�����Ĵ�������

  while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));
   xsb=I2C_ReceiveData(I2C1);//�����Ĵ�������

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

   I2C_GenerateSTART(I2C1,ENABLE);//��ʼ�ź�
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

   I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Transmitter);//�����豸��ַ+д�ź�
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//  

   I2C_Cmd(I2C1,ENABLE);
   I2C_SendData(I2C1,reg_Addr);//���ʹ洢��Ԫ��ַ����0��ʼ
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

   I2C_GenerateSTART(I2C1,ENABLE);//��ʼ�ź�

   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
   I2C_Send7bitAddress(I2C1,I2C_Addr,I2C_Direction_Receiver);//�����豸��ַ+���ź�

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
	     *pBuff = I2C_ReceiveData(I2C1);//�����Ĵ�������

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

