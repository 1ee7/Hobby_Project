/******************** (C) COPYRIGHT 2012  **************************

 * 硬件连接：-----------------
 *          |                 |
 *          |  PB6-I2C1_SCL		|
 *          |  PB7-I2C1_SDA   |
 *          |                 |
 *           -----------------
 * 库版本  ：ST3.5.0
 * 作者    ： Orange 
**********************************************************************************/
#include "mpu6050.h"


 /*
 * 函数名：Key_GPIO_Config
 * 描述  ：Key I/O配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
		/* 使能与 的时钟 */
	RCC_APB2PeriphClockCmd  (RCC_APB2Periph_GPIOA,ENABLE ); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //下拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	//GPIO_ResetBits(GPIOA,GPIO_Pin_0);
}





/***************************************************************************************/
/*
 * 函数名：void MPU6050_Init(void)
 * 描述  ：初始化Mpu6050
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void MPU6050_Init(void)
{	   
	Key_GPIO_Config(); //校准按键
	
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,PWR_MGMT_1,0x00);//解除休眠状态
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,SMPLRT_DIV,0x07);
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,CONFIG,0x06);
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,GYRO_CONFIG,0x08);//500 度/s
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,ACCEL_CONFIG,0x08); //4g


	I2C_ByteWrite_Reg(MPU6050_I2C_Address,MPU_USER_CTRL_REG,0x00);	//I2C主模式关闭
//	I2C_ByteWrite_Reg(MPU6050_I2C_Address,MPU_FIFO_EN_REG,0x00);	//关闭FIFO
	I2C_ByteWrite_Reg(MPU6050_I2C_Address,MPU_INTBP_CFG_REG,0x02);	//INT引脚低电平有效(0x80),IIC设置为旁路模式(0x02) *原子源码为0x80

}  


/*
 * 函数名：GetData
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 */

s16 GetData(unsigned char REG_Address)
{
	char H,L;
	H=I2C_ByteRead(MPU6050_I2C_Address,REG_Address);
	L=I2C_ByteRead(MPU6050_I2C_Address,REG_Address+1);
	return (s16)(H<<8)+L;   //合成数据
}



/******************* (C) COPYRIGHT 2012  *****END OF FILE************/






			



