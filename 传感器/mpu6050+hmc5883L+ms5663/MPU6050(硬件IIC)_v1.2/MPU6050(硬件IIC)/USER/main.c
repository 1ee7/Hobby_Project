/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * 文件名  ：main.c
 * 描述    ：I2C EEPROM(AT24C02)测试，测试信息通过USART1打印在电脑的超级终端。
 *          
 * 实验平台：野火STM32开发板
 * 库版本  ：ST3.0.0
 *
 * 作者    ：wildfire team 
 * 论坛    ：http://www.amobbs.com/forum-1008-1.html
 * 淘宝    ：http://firestm32.taobao.com
**********************************************************************************/	
#include "stm32f10x.h"
#include "mpu6050.h"
#include "usart1.h"
#include "delay.h"
#include <math.h>
#include "ms5611.h"

#include "hmc5883l.h"
//======卡尔曼滤波======//
//float dt=0.001;//注意：dt的取值为kalman滤波器采样时间

float P[2][2] = {
                 { 1, 0 },
                 { 0, 1 }
				};
float Pdot[4] ={ 0,0,0,0};
float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度
float R_angle=0.5 ,C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

//========四元数 =====//
#define Kp 100.0 // 比例增益支配率收敛到加速度计/ 磁强计
#define Ki 0.002 // 积分增益支配率的陀螺仪偏见的衔接
#define halfT 0.001 //采样周期的一半

float q0=1,q1=0,q2=0,q3=0;  //四元数的元素，代表估计的方向
float exInt = 0,eyInt = 0,ezInt=0;   //按比例缩小积分误差
float Yaw,Pitch,Roll; //偏航角，俯仰角,翻滚角

//======一阶互补滤波====//
float K1=0.1;     //对加速度计取值的权重
float dt=0.01;	  //dt的取值为滤波器采样时间

float calculateTilt(float ax,float ay,float az,char flag_x,char flag_y,char flag_z);

 //加速度
u8 Index_Ac_RawData[3]={
    ACCEL_XOUT_H,
	ACCEL_YOUT_H,
	ACCEL_ZOUT_H,
  };
 //陀螺仪 角速度
u8 Index_Gy_RawData[3]={
    GYRO_XOUT_H,
	GYRO_YOUT_H,
	GYRO_ZOUT_H,
  };
  
float K_Ac_Value[3]={0,0,0};   //加速度的偏移量
float K_Gy_Value[3]={0,0,0};   //角速度的偏移量				  

float Value_Ac_RawData[3];
float Value_Gy_RawData[3];

s16 Ac_RawData[3];
s16 Gy_RawData[3]; 

u32 sum=0;

extern int g_MagHead[3];

void HandleRawData(void);
void GetBalance(void);
void yijiehubuPitch_init(void);
void yijiehubuRoll_init(void);
void IMUupdate(float gx,float gy,float gz,float ax,float ay,float az);

void Kalam_getData(void);


long g_temperature,g_pressure;
float g_MagYaw;
/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 * 返回  ：无
 */
int main(void)
{	
//	float angel_title;
    uint8_t i;
    float temp;
	float hight;
	/* 延时 systick 初始化 */
	delay_init();	

	/* 串口1初始化 */
	USART1_Config(115200);
	
	/*IIC接口初始化*/
	InitI2C();	
		 
	/*陀螺仪传感器初始化*/
    MPU6050_Init();	

	/* 气压计初始化 */
    MS5611_Init(); 

	/* 磁力计初始化 */
    HMC5883l_Init();
    //HMC5883L_Initialize();	

	/***********************************************************************/
	
	while(1)
	{	  
	   if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1))
	   {
	   printf("\r\n 开始准备。。。\r\n");
	      for(i=0;i<100;i++)
		     delay_ms(200);

	       printf("MPu6050校准 开始 ....\r\n");
		   GetBalance();
		   printf("MPu6050校准 完成 ....\r\n");
		   calibrateMag(); //磁力计校准

		   for(i=0;i<100;i++)
		     delay_ms(200);	
		 printf("\r\n数据采集开始\r\n");	 	  

	   }
	    
	
      /* MS5611 */
      
	    MS5611_Convert(&g_temperature,&g_pressure);
       // printf("tmp: %ld,press: %ld\r\n",g_temperature, g_pressure);
	    temp = g_temperature/100.0;
	    hight = 8000*((1+temp/273.0)/g_pressure);
	  //  printf("temp : %f , hight :%f \r\n", temp,hight);
		
	  /*   MPU6050  */	  
         HandleRawData();
	  /*  HMC5883L */
		g_MagYaw=Get_yaw();
	//	printf("magnetometer is %f \r\n",g_MagYaw);
	   //  printf("%f %f %f %f %f %f\n",
	   //	                             Value_Ac_RawData[0],Value_Ac_RawData[1],Value_Ac_RawData[2],
	   //								 Value_Gy_RawData[0],Value_Gy_RawData[1],Value_Gy_RawData[2]
	   //								 );
     //::::::::::卡尔曼::::::::::::::://
       // Kalam_getData();

	 //::::::::::::四元数:::::::::::::://

   	    IMUupdate(Value_Gy_RawData[0],Value_Gy_RawData[1],Value_Gy_RawData[2],
	          Value_Ac_RawData[0],Value_Ac_RawData[1],Value_Ac_RawData[2]);
	
	 //::::::::::::一阶滤波::::::::::://
	   //  yijiehubuPitch_init();
	   //  yijiehubuRoll_init();
	   //angel_title=calculateTilt(Value_Gy_RawData[0],Value_Gy_RawData[1],Value_Gy_RawData[2],1,1,0);

       //  printf("title is %f \r\n",angel_title);

	   Yaw = g_MagYaw;  
	  // Yaw = atan2f(Value_Ac_RawData[0]*g_MagHead[2] - Value_Ac_RawData[2]*g_MagHead[0],
	    //            Value_Ac_RawData[2]*g_MagHead[1] - Value_Ac_RawData[1]*g_MagHead[2]);
	   printf("%f %f %f\n",Roll,Pitch,Yaw );
        
	/*  
	    printf("::::::::::::::::::::::: 加速度 :::::::::::::::::::::::\r\n");
		printf("\r\n---------加速度X轴原始数据---------%d \r\n",GetData(ACCEL_XOUT_H));
		printf("\r\n---------加速度Y轴原始数据---------%d \r\n",GetData(ACCEL_YOUT_H));	
		printf("\r\n---------加速度Z轴原始数据---------%d \r\n",GetData(ACCEL_ZOUT_H));	

		printf("::::::::::::::::::::::: 陀螺仪 :::::::::::::::::::::::\r\n");
		printf("\r\n---------陀螺仪X轴原始数据---------%d \r\n",GetData(GYRO_XOUT_H));	
		printf("\r\n---------陀螺仪Y轴原始数据---------%d \r\n",GetData(GYRO_YOUT_H));	
		printf("\r\n---------陀螺仪Z轴原始数据---------%d \r\n",GetData(GYRO_ZOUT_H));
	*/
    	delay_ms(200);
	}		
	  
}

/*
 * 函数名：HandleRawData
 * 描述  ：处理数据
 * 输入  ：无
 * 输出  ：无
 * 返回  ：无
 */
void HandleRawData(void)
{
  u8 i;

  for(i=0;i<3;i++)
  {	
     Ac_RawData[i]  = GetData(Index_Ac_RawData[i]);
	 Gy_RawData[i]  = GetData(Index_Gy_RawData[i]);

	 //陀螺仪  角速度	 有效值
     Value_Gy_RawData[i] =  (float)((float)(Gy_RawData[i])*GY_RANGE/TOTAL_RANGE)-K_Gy_Value[i];
    //加速度	 有效值
	 Value_Ac_RawData[i] =  (float)((float)(Ac_RawData[i])*AC_RANGE/TOTAL_RANGE)-K_Ac_Value[i];

	 //printf(" %d ---> 加速度 %d -%f- %f , 陀螺仪角速度%d -%f- %f \r\n",i,Ac_RawData[i],(float)Ac_RawData[i],Value_Ac_RawData[i],
	 //                                                          Gy_RawData[i],(float)Gy_RawData[i],Value_Gy_RawData[i]) ;
 	
  }
}

/*
 * 函数名：GetBalance
 * 描述  ：校准
 * 输入  ：无
 * 输出  ：无
 * 返回  ：无
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
   sum_ac[2]=1;//g 重力加速度

   for(j=0;j<3;j++)
  {
    K_Ac_Value[j] = (float)(sum_ac[j]/10.0); 
	K_Gy_Value[j] = (float)(sum_gy[j]/10.0);
  }

//  printf("校准值为 加速度 %f,%f,%f,  角速度 %f,%f,%f \r\n",K_Ac_Value[0],K_Ac_Value[1],K_Ac_Value[2],
//                                                           K_Gy_Value[0],K_Ac_Value[1],K_Ac_Value[2]);	
//
}

//::::::::::::::::::四元数算法:::::::::::::::::::://
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
//积分误差比例积分增益
   exInt = exInt + ex*Ki;
   eyInt = eyInt + ey*Ki;
   ezInt = ezInt + ez*Ki;
//调整后的陀螺仪测量
   gx = gx + Kp*ex + exInt;
   gy = gy + Kp*ey + eyInt;
   gz = gz + Kp*ez + ezInt;
//整合四元数率和正常化
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

//正常化四元
    norm = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0 = q0/norm;
	q1 = q1/norm;
	q2 = q2/norm;
	q3 = q3/norm;

	Pitch = asin(-2*q1*q3+2*q0*q2)*57.3;
	Roll = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.3;
	Yaw = atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;
 
}

//:::::::::::::::一阶互补滤波:::::::::::::://
float yijiehubu(float angle_m,float gyro_m)
{ 
   float angle;
   angle = K1*angle_m+(1-K1)*(angle+gyro_m*dt);
   return angle;
}



 void yijiehubuPitch_init(void)
 {
  float angle_ax;
  //angle_ax = atan(Value_Ac_RawData[1]/Value_Ac_RawData[2])*57.3;  //加速度得到的角度
  angle_ax = asin(Value_Ac_RawData[1]/Value_Ac_RawData[2])*57.3;  //加速度得到的角度
  Pitch = yijiehubu(angle_ax,Value_Gy_RawData[1]);
 }


 void yijiehubuRoll_init(void)
 {
 	float angle_ay;
   //angle_ay = atan(Value_Ac_RawData[0]/Value_Ac_RawData[2])*57.3;  //加速度得到的角度
    angle_ay = asin(Value_Ac_RawData[0]/Value_Ac_RawData[2])*57.3;  //加速度得到的角度
    Roll = yijiehubu(angle_ay,Value_Gy_RawData[0]);
}
//::::::::::::::::::::卡尔曼滤波:::::::::::::::::::://



float Kalman_Filter(float angle_m, float gyro_m)	//angleAx 和 gyroGy
{
       float angle;
	   float angle_dot;//角度和角速度

        angle+=(gyro_m-q_bias) * dt;
        angle_err = angle_m - angle;
        Pdot[0]=Q_angle - P[0][1] - P[1][0];
        Pdot[1]=- P[1][1];
        Pdot[2]=- P[1][1];
        Pdot[3]=Q_gyro;
        P[0][0] += Pdot[0] * dt;
        P[0][1] += Pdot[1] * dt;
        P[1][0] += Pdot[2] * dt;
        P[1][1] += Pdot[3] * dt;
        PCt_0 = C_0 * P[0][0];
        PCt_1 = C_0 * P[1][0];
        E = R_angle + C_0 * PCt_0;
        K_0 = PCt_0 / E;
        K_1 = PCt_1 / E;
        t_0 = PCt_0;
        t_1 = C_0 * P[0][1];
        P[0][0] -= K_0 * t_0;
        P[0][1] -= K_0 * t_1;
        P[1][0] -= K_1 * t_0;
        P[1][1] -= K_1 * t_1;
        angle += K_0 * angle_err; //最优角度
        q_bias += K_1 * angle_err;
        angle_dot = gyro_m-q_bias;//最优角速度
        return angle;
}

void Kalam_getData(void)
{
   float angle_ax,angle_ay,angle_az;
   angle_ax = atan(Value_Ac_RawData[1]/Value_Ac_RawData[2])*57.3*10;  //加速度得到的角度
   angle_ay = atan(Value_Ac_RawData[0]/Value_Ac_RawData[2])*57.3*10;
   angle_az = atan(Value_Ac_RawData[0]/Value_Ac_RawData[1])*57.3*10;
   Pitch =  Kalman_Filter(angle_ax,Value_Gy_RawData[0]);
   Roll  =  Kalman_Filter(angle_ay,Value_Gy_RawData[1]);
   Yaw =    Kalman_Filter(angle_az,Value_Gy_RawData[2]);

}

//:::::::::::::::::::三轴加速度计数据计算倾斜角算法:::::::::::::::://
float calculateTilt(float ax,float ay,float az,char flag_x,char flag_y,char flag_z)
{
  float g=9.80665;
  float temp;
  float Tiltangle = 0;
  temp = ((sqrt(2)/2)*g/10);
  if(az>((sqrt(2)/2)*g/10))
  {
  	Tiltangle = (1-ay*ay)-(1-ax*ax);
	if(Tiltangle<0)
	 Tiltangle = -Tiltangle;
	Tiltangle = acos(sqrt(Tiltangle));
	Tiltangle = Tiltangle/3.1415*180;
	if(flag_x == 1 || flag_y == 1)
	{
		Tiltangle += 90;
	}
	else
	{
		Tiltangle = 90-Tiltangle;
	}
  }
  else
  {
    Tiltangle = asin(az);
	Tiltangle = Tiltangle/3.1415*180;
	if(flag_z == 1)
	{
	  Tiltangle += 90;
	}
	else
	{
	  Tiltangle = 90 - Tiltangle;
	}

  }

   return Tiltangle;
}
//::::::::::::::: timer 2 ::::::::::::::::::::://

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
