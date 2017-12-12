#include "hmc5883l.h"
#include <math.h>

#define MagnetDeclination 4.43
#define CalThreshold 0

uint8_t HMC5883Lmode;
int g_MagData[3];
int g_MagHead[3];
int g_MagMax[3];
int g_MagMin[3];

int offsetX,offsetY,offsetZ;

#define PI 3.141592654f  

void HMC5883l_Init(void)
{
   // write CONFIG_A register
    hmc5883lWriteByte(0x3c,0x70/* 0x6c*/ , 0x00); //数据测量、输出速率15hz  注意，此处会影响数值的准确性

   // write CONFIG_B register
//    hmc5883lWriteByte(0x3c, 0x00, 0x01);

   // write CONFIG_C register
    hmc5883lWriteByte(0x3c, 0x00, 0x02);   //连续模式 
	

}

void HMC5883L_SetGain(uint8_t gain)
{
    // use this method to guarantee that bits 4-0 are set to zero, which is a
    // requirement specified in the datasheet; 
    uint8_t tmp = gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1);
    hmc5883lWriteByte(HMC5883L_DEFAULT_ADDRESS, tmp, HMC5883L_RA_CONFIG_B);
}

/** Set measurement mode.
 * @param newMode New measurement mode
 * @see HMC5883L_GetMode()
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
void HMC5883L_SetMode(uint8_t newMode)
{
    // use this method to guarantee that bits 7-2 are set to zero, which is a
    // requirement specified in the datasheet; 
    uint8_t tmp = HMC5883Lmode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1);
    hmc5883lWriteByte(HMC5883L_DEFAULT_ADDRESS, tmp, HMC5883L_RA_MODE);
    HMC5883Lmode = newMode; // track to tell if we have to clear bit 7 after a read
}

 void HMC5883L_Initialize(void)
{
    // write CONFIG_A register

    uint8_t tmp = (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1))
            | (HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1))
            | (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1));
    hmc5883lWriteByte(HMC5883L_DEFAULT_ADDRESS, tmp, HMC5883L_RA_CONFIG_A);

    // write CONFIG_B register
    HMC5883L_SetGain(HMC5883L_GAIN_1090);

    // write MODE register
    HMC5883L_SetMode(HMC5883L_MODE_SINGLE);
}


void HMC5883l_GetData(void)
{

  static uint8_t XYZ_Data[6];

 // uint8_t id[3];

//	id[0]=I2C_ByteRead(0x3d,10);
//	id[1]=I2C_ByteRead(0x3d,11);
//	id[2]=I2C_ByteRead(0x3d,12);

   // hmc5883lRead(0x3d,0x11,id,2);
//	printf("\r\n id is %x,%x,%x \r\n",id[0],id[1],id[2]);
   	XYZ_Data[0] = I2C_ByteRead(0x3d,3);
	XYZ_Data[1] = I2C_ByteRead(0x3d,4);
	XYZ_Data[2] = I2C_ByteRead(0x3d,5);
	XYZ_Data[3] = I2C_ByteRead(0x3d,6);
	XYZ_Data[4] = I2C_ByteRead(0x3d,7);
	XYZ_Data[5] = I2C_ByteRead(0x3d,8);


   g_MagData[0]=((uint16_t)XYZ_Data[0]<<8)|XYZ_Data[1];
   g_MagData[1]=((uint16_t)XYZ_Data[2]<<8)|XYZ_Data[3];
   g_MagData[2]=((uint16_t)XYZ_Data[4]<<8)|XYZ_Data[5];
	
   if(g_MagData[0]>0x7fff) g_MagData[0]-=0xffff;
   if(g_MagData[1]>0x7fff) g_MagData[1]-=0xffff;
   if(g_MagData[2]>0x7fff) g_MagData[2]-=0xffff;
 
} 


float Get_yaw(void)
{
	  int offsetX=0,offsetY=0,offsetZ=0;      //定义零偏值
	  float yaw_hmc;

      
//    [倾角补偿]    	
//	  float ctpx,stpx,ctpy,stpy;        //计算角度	

//    [倾角补偿]未作数据融合,注释	
//	  ctpx=cos((float)pitch*PI/180);   //俯仰角,翻滚角转换为弧度(除以57.3)
//	  stpx=sin((float)pitch*PI/180);   
//	  ctpy=cos((float)roll*PI/180);    
//	  stpy=sin((float)roll*PI/180);	   
		
	  //磁力计的校正值
	/*
	  g_MagMax[0]=294; //455
	  g_MagMax[1]=292; //357
	  g_MagMax[2]=331; //311

	  g_MagMin[0]=-502; //-520
	  g_MagMax[1]=-524; //-599
	  g_MagMax[2]=-419; //-599    
   
	  //计算各轴的零偏
	   offsetX =(g_MagMax[0]+g_MagMin[0])/2;
	   offsetY =(g_MagMax[1]+g_MagMin[1])/2;
	   offsetZ =(g_MagMax[2]+g_MagMin[2])/2;
	*/	
	  //读取磁力计的原始数据	
	   HMC5883l_GetData();
	
	  //计算各轴的heading值 [heading值=原始数据-该轴零偏值]
	  
	  g_MagHead[0]=g_MagData[0]-offsetX; 
	  g_MagHead[1]=g_MagData[1]-offsetY;
	  g_MagHead[2]=g_MagData[2]-offsetZ;
	   

    // printf("\r\n gHead %d,%d,%d \r\n",g_MagHead[0],g_MagHead[1],g_MagHead[2]);
//   [倾角补偿]等价于水平情况 heading原校正后原始数据改为mag.heading.x
//	  mag.out.x=(int)((float)mag.heading.x*ctpx+(float)mag.heading.y*stpy*stpx-(float)mag.heading.z*ctpy*stpx);
//	  mag.out.y=(int)((float)mag.heading.y*ctpy+(float)mag.heading.z*stpy);
	  
	 yaw_hmc=atan2((float)(g_MagHead[1]),(float)(g_MagHead[0]))/*(180.0/PI)*/;   //计算得出YAW角 yaw=arc tan(前/后)
	//  yaw_hmc=2*atan((float)(g_MagHead[1])/(float)(g_MagHead[0]))*(180.0/PI);
	// printf("tgl debug yaw %f \r\n",yaw_hmc);
	if(yaw_hmc<0)
	  	yaw_hmc += 2*PI;
	yaw_hmc = yaw_hmc*(180.0/PI)+MagnetDeclination;
	if(yaw_hmc > 360)
	  yaw_hmc -= 360;
	/* 
	  g_MagHead[0]=(float)g_MagData[0]*0.92; 
	  g_MagHead[1]=(float)g_MagData[1]*0.92;
	  g_MagHead[2]=(float)g_MagData[2]*0.92;

	  yaw_hmc = atan2(g_MagHead[1],g_MagHead[0]);
	 
	  if(yaw_hmc<0)
	    yaw_hmc += 2*PI;
	  if(yaw_hmc > 2*PI)
	    yaw_hmc -= 2*PI;
	  
		yaw_hmc = yaw_hmc*180/PI;
	*/
	  return yaw_hmc;
}


//校准
void calibrateMag(void)
{
	uint8_t i;
//	int x,y,z;
	int xMax,xMin,yMax,yMin,zMax,zMin;
   printf("\r\n 磁力计校准.....\r\n");
	HMC5883l_GetData();

	xMax = xMin = g_MagData[0];
	yMax = yMin = g_MagData[1];
	zMax = zMin = g_MagData[2];

	offsetX = offsetY = offsetZ = 0;

	for(i=0;i<200;i++)
	{
	  HMC5883l_GetData();

	  if(g_MagData[0]>xMax)
         xMax = g_MagData[0];
	  if(g_MagData[0] <xMin)
	     xMin = g_MagData[0];

	  if(g_MagData[1]>yMax)
         yMax = g_MagData[1];
	  if(g_MagData[1] <yMin)
	     yMin = g_MagData[1];
	  
	  if(g_MagData[2]>zMax)
         zMax = g_MagData[2];
	  if(g_MagData[2] <zMin)
	     zMin = g_MagData[2];

	    delay_ms(100);
		if(i%10 == 0)
		{
		  printf("max-%d,min-%d\r\n",xMax,xMin);
		}
	 }

	 

	  if(abs(xMax-xMin)>CalThreshold)
	    offsetX = (xMax+xMin)/2;

	   if(abs(yMax-yMin)>CalThreshold)
	    offsetY = (yMax+yMin)/2;

	   if(abs(zMax-zMin)>CalThreshold)
	    offsetZ = (zMax+zMin)/2;
	delay_ms(800);
	 printf("\r\n offset(%d,%d,%d)\r\n",offsetX,offsetY,offsetZ); 
	   printf("\r\n 磁力计校准完成。\r\n");
}
