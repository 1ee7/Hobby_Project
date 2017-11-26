/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * �ļ���  ��main.c
 * ����    ��I2C EEPROM(AT24C02)���ԣ�������Ϣͨ��USART1��ӡ�ڵ��Եĳ����նˡ�
 *          
 * ʵ��ƽ̨��Ұ��STM32������
 * ��汾  ��ST3.0.0
 *
 * ����    ��wildfire team 
 * ��̳    ��http://www.amobbs.com/forum-1008-1.html
 * �Ա�    ��http://firestm32.taobao.com
**********************************************************************************/	
#include "stm32f10x.h"
#include "I2C_MPU6050.h"
#include "usart1.h"
#include "delay.h"
#include <math.h>


//========��Ԫ�� =====//
#define Kp 100.0 // ��������֧�������������ٶȼ�/ ��ǿ��
#define Ki 0.002 // ��������֧���ʵ�������ƫ�����ν�
#define halfT 0.001 //�������ڵ�һ��

float q0=1,q1=0,q2=0,q3=0;  //��Ԫ����Ԫ�أ�������Ƶķ���
float exInt = 0,eyInt = 0,ezInt=0;   //��������С�������
float Yaw,Pitch,Roll; //ƫ���ǣ�������,������

//======һ�׻����˲�====//
float K1=0.1;     //�Լ��ٶȼ�ȡֵ��Ȩ��
float dt=0.01;	  //dt��ȡֵΪ�˲�������ʱ��



 //���ٶ�
u8 Index_Ac_RawData[3]={
    ACCEL_XOUT_H,
	ACCEL_YOUT_H,
	ACCEL_ZOUT_H,
  };
 //������ ���ٶ�
u8 Index_Gy_RawData[3]={
    GYRO_XOUT_H,
	GYRO_YOUT_H,
	GYRO_ZOUT_H,
  };
  
float K_Ac_Value[3]={0,0,0};   //���ٶȵ�ƫ����
float K_Gy_Value[3]={0,0,0};   //���ٶȵ�ƫ����				  

float Value_Ac_RawData[3];
float Value_Gy_RawData[3];

s16 Ac_RawData[3];
s16 Gy_RawData[3]; 

u32 sum=0;

void HandleRawData(void);
void GetBalance(void);
void yijiehubuPitch_init(void);
void yijiehubuRoll_init(void);
/*
 * ��������main
 * ����  ��������
 * ����  ����
 * ���  ����
 * ����  ����
 */
int main(void)
{	


	/* ����1��ʼ�� */
	USART1_Config(115200);
	
	/*IIC�ӿڳ�ʼ��*/
	I2C_MPU6050_Init(); 
		 
	/*�����Ǵ�������ʼ��*/
    InitMPU6050();
	

	/***********************************************************************/
	
	while(1)
	{	 
	   	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1))
		{
		    printf("У׼ ��ʼ ....\r\n");
		   GetBalance();
		}
	      
		HandleRawData();
	  //  printf("%f %f %f %f %f %f\n",
	  //	                             Value_Ac_RawData[0],Value_Ac_RawData[1],Value_Ac_RawData[2],
	  //								 Value_Gy_RawData[0],Value_Gy_RawData[1],Value_Gy_RawData[2]
	  //								 );

	   yijiehubuPitch_init();
	   yijiehubuRoll_init();
	 //  printf("\r\n ===============\r\n Pitch: %f\r\n",Pitch);
	   printf("%f %f %f\n",Roll,Pitch,Yaw );
        
	/*  
	    printf("::::::::::::::::::::::: ���ٶ� :::::::::::::::::::::::\r\n");
		printf("\r\n---------���ٶ�X��ԭʼ����---------%d \r\n",GetData(ACCEL_XOUT_H));
		printf("\r\n---------���ٶ�Y��ԭʼ����---------%d \r\n",GetData(ACCEL_YOUT_H));	
		printf("\r\n---------���ٶ�Z��ԭʼ����---------%d \r\n",GetData(ACCEL_ZOUT_H));	

		printf("::::::::::::::::::::::: ������ :::::::::::::::::::::::\r\n");
		printf("\r\n---------������X��ԭʼ����---------%d \r\n",GetData(GYRO_XOUT_H));	
		printf("\r\n---------������Y��ԭʼ����---------%d \r\n",GetData(GYRO_YOUT_H));	
		printf("\r\n---------������Z��ԭʼ����---------%d \r\n",GetData(GYRO_ZOUT_H));
		*/
		delay_ms(10);
	}		

	 
 			  
}

/*
 * ��������HandleRawData
 * ����  ����������
 * ����  ����
 * ���  ����
 * ����  ����
 */
void HandleRawData(void)
{
  u8 i;

  for(i=0;i<3;i++)
  {	
     Ac_RawData[i]  = GetData(Index_Ac_RawData[i]);
	 Gy_RawData[i]  = GetData(Index_Gy_RawData[i]);

	 //������  ���ٶ�	 ��Чֵ
     Value_Gy_RawData[i] =  (float)((float)(Gy_RawData[i])*GY_RANGE/TOTAL_RANGE)-K_Gy_Value[i];
    //���ٶ�	 ��Чֵ
	 Value_Ac_RawData[i] =  (float)((float)(Ac_RawData[i])*AC_RANGE/TOTAL_RANGE)-K_Ac_Value[i];

	 //printf(" %d ---> ���ٶ� %d -%f- %f , �����ǽ��ٶ�%d -%f- %f \r\n",i,Ac_RawData[i],(float)Ac_RawData[i],Value_Ac_RawData[i],
	   //                                                          Gy_RawData[i],(float)Gy_RawData[i],Value_Gy_RawData[i]) ;
 	
  }
}

/*
 * ��������GetBalance
 * ����  ��У׼
 * ����  ����
 * ���  ����
 * ����  ����
 */
void GetBalance(void)
{
   u8 i,j;
   float sum_ac[3]={0,0,0},sum_gy[3]={0,0,0};
   s16 Tmp_Ac_RawData[3];
   s16 Tmp_Gy_RawData[3]; 

   for(j=0;j<3;j++)
   {
	 sum_ac[j]=0;
	 sum_gy[j]=0;
   }
   for(i=0;i<10;i++)
   {  
    for(j=0;j<3;j++)

      {
	    Tmp_Ac_RawData[j]  = GetData(Index_Ac_RawData[j]);
	    Tmp_Gy_RawData[j]  = GetData(Index_Gy_RawData[j]);
	    
	    sum_gy[j] +=  (float)((float)(Tmp_Gy_RawData[j])*GY_RANGE/TOTAL_RANGE);
	    sum_ac[j] +=  (float)((float)(Tmp_Ac_RawData[j])*AC_RANGE/TOTAL_RANGE);
	  }	 	  
	 
   } 
   sum_ac[2]=1;//g �������ٶ�

   for(j=0;j<3;j++)
  {
    K_Ac_Value[j] = (float)(sum_ac[j]/10.0); 
	K_Gy_Value[j] = (float)(sum_gy[j]/10.0);
  }

//  printf("У׼ֵΪ ���ٶ� %f,%f,%f,  ���ٶ� %f,%f,%f \r\n",K_Ac_Value[0],K_Ac_Value[1],K_Ac_Value[2],
//                                                           K_Gy_Value[0],K_Ac_Value[1],K_Ac_Value[2]);	
//
}

//::::::::::::::::::��Ԫ���㷨:::::::::::::::::::://
void IMUupdate(float gx,float gy,float gz,float ax,float ay,float az)
{
   float norm;
   float vx,vy,vz;
   float ex,ey,ez;

   norm = sqrt(ax*ax + ay*ay + az*az);
   ax = ax/norm;
   ay = ay/norm;
   az = az/norm;

   vx = 2*(q1*q3 - q0*q2);
   vy = 2*(q0*q1 + q2*q3);
   vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

   ex = (ay*vz - az * vy);
   ey = (az*vx - ax * vz);
   ez = (ax*vy - ay * vx);
//������������������
   exInt = exInt + ex*Ki;
   eyInt = eyInt + ey*Ki;
   ezInt = ezInt + ez*Ki;
//������������ǲ���
   gx = gx + Kp*ex + exInt;
   gy = gy + Kp*ey + eyInt;
   gz = gz + Kp*ez + ezInt;
//������Ԫ���ʺ�������
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

//��������Ԫ
    norm = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0 = q0/norm;
	q1 = q1/norm;
	q2 = q2/norm;
	q3 = q3/norm;

	Pitch = asin(-2*q1*q3+2*q0*q2)*57.3;
	Roll = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.3;
 
}

//:::::::::::::::һ�׻����˲�:::::::::::::://
float yijiehubu(float angle_m,float gyro_m)
{ 
   float angle;
   angle = K1*angle_m+(1-K1)*(angle+gyro_m*dt);
   return angle;
}



 void yijiehubuPitch_init(void)
 {
  float angle_ax;
  angle_ax = atan(Value_Ac_RawData[1]/Value_Ac_RawData[2])*57.3;  //���ٶȵõ��ĽǶ�
  Pitch = yijiehubu(angle_ax,Value_Gy_RawData[1]);
 }


 void yijiehubuRoll_init(void)
 {
 	float angle_ay;
    angle_ay = atan(Value_Ac_RawData[0]/Value_Ac_RawData[2])*57.3;  //���ٶȵõ��ĽǶ�
    Roll = yijiehubu(angle_ay,Value_Gy_RawData[0]);
}
 



/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
