/******************** (C) COPYRIGHT 2012  **************************

 * Ӳ�����ӣ�-----------------
 *          |                 |
 *          |  PB6-I2C1_SCL		|
 *          |  PB7-I2C1_SDA   |
 *          |                 |
 *           -----------------
 * ��汾  ��ST3.5.0
 * ����    �� Orange 
**********************************************************************************/
#include "mpu6050.h"


 /*
 * ��������Key_GPIO_Config
 * ����  ��Key I/O����
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
		/* ʹ���� ��ʱ�� */
	RCC_APB2PeriphClockCmd  (RCC_APB2Periph_GPIOA,ENABLE ); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	//GPIO_ResetBits(GPIOA,GPIO_Pin_0);
}





/***************************************************************************************/
/*
 * ��������void MPU6050_Init(void)
 * ����  ����ʼ��Mpu6050
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void MPU6050_Init(void)
{	   
	Key_GPIO_Config(); //У׼����
	
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,PWR_MGMT_1,0x00);//�������״̬
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,SMPLRT_DIV,0x07);
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,CONFIG,0x06);
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,GYRO_CONFIG,0x08);//500 ��/s
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,ACCEL_CONFIG,0x08); //4g


	I2C_ByteWrite_Reg(MPU6050_I2C_Address,MPU_USER_CTRL_REG,0x00);	//I2C��ģʽ�ر�
//	I2C_ByteWrite_Reg(MPU6050_I2C_Address,MPU_FIFO_EN_REG,0x00);	//�ر�FIFO
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,MPU_INTBP_CFG_REG,0x02);	//INT���ŵ͵�ƽ��Ч(0x80),IIC����Ϊ��·ģʽ(0x02) *ԭ��Դ��Ϊ0x80

}  


/*
 * ��������GetData
 * ����  �����16λ����
 * ����  ��REG_Address �Ĵ�����ַ
 * ���  �����ؼĴ�������
 * ����  ���ⲿ����
 */

s16 GetData(unsigned char REG_Address)
{
	char H,L;
	H=I2C_ByteRead(MPU6050_I2C_Address,REG_Address);
	L=I2C_ByteRead(MPU6050_I2C_Address,REG_Address+1);
	return (s16)(H<<8)+L;   //�ϳ�����
}



/******************* (C) COPYRIGHT 2012  *****END OF FILE************/






			



