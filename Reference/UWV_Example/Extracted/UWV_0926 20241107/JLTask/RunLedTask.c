#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "semphr.h"

#include "JLGpio.h"
#include "gpio.h"
#include "JointTask.h"
#include "CanMsg.h"
#include "usartdma.h"
#include "string.h"
#include "kinematics.h"
#include "math.h" 

#include "usartdma.h"
#include "CanMsg.h"
#include  "arm_math.h"
#include "stdio.h"

//-----------遥控器宏定义------------------------------------
#define DEADZONE_SLOPE				8.925							//死区斜率
#define DEADZONE_BOUND				340								//死区边界值
#define SPEED_MAX							6804							//电机最大电转速
#define SPEED_MIN 						-6804							//电机最小电转速
#define CH_MAX								0x0708						//遥控器通道最大值1800
#define	CH_MID								0x03E8						//遥控器通道中间值1000
#define CH_MIN								0x00C8						//遥控器通道最小值200
#define UART1_SEND_ENABLE			0									//串口发送使能
#define ODO_Rate							0.00022321f				//电机里程分辨率（实际里程(m) / 电机里程）
#define LVDAIJULI							0.77f * 0.895f							//履带车履带中点距离
//-----------电机参数-------------------------------------------
float real_odo1 = 0.0;
float real_odo2 = 0.0;
int32_t rpmBuf[2] = {0, 0};
struct kinematic_model xy;
//-----------陀螺仪参数-----------------------------------------
float angX,angY,angZ,angZZ;
//--------------------------------------------------------------

// //这边对输出PWM波做处理。
// PWM1    PE9    TIM1_CH1
//PWM2    PE11  TIM1_CH2
//PWM3    PE13  TIM1_CH3
//PWM4    PE14  TIM1_CH4
//PWM5    PD12  TIM4_CH1  //冲洗水枪     //900-2400。
//PWM6    PD13  TIM4_CH2  //水下磨刷900-2200pwm波
//PWM7    PD14  TIM4_CH3 // 水下钻头900-1500-2200pwm波
//PWM8    PD15  TIM4_CH4 //水下切割900-2400
//PWM9    PC6   TIM3_CH1 
//PWM10  PC7   TIM3_CH2		//控制摄像头灯，开和关。
//PWM11  PC8   TIM3_CH3    控制材料挤出机900-1500 1500-2000
//PWM12  PC9   TIM3_CH4
// ADC转换的电压值通过MDA方式传到SRAM
extern __IO uint16_t ADC_ConvertedValue[RHEOSTAT_NOFCHANEL];

// 局部变量，用于保存转换计算后的电压值 	 
float ADC_ConvertedValueLocal[RHEOSTAT_NOFCHANEL]={0}; 
 
uint16_t t4ccr1=900;
uint16_t t4ccr2=1500; //900
uint16_t t4ccr3=1500;
uint16_t t4ccr4=900;
uint16_t t3ccr3=1500;
uint16_t djCURRENT=0;	
uint16_t djCURRENTlast=0;

//-------履带车运动角度位置计算-----------------------------
void solve_kinematic_model(void)
{
	memcpy(xy.odo_last, xy.odo_now, 8);
	memcpy(xy.odo_now, odoBuf, 8);
	xy.odo_inc[0] = xy.odo_now[0] - xy.odo_last[0];
	xy.odo_inc[1] = xy.odo_now[1] - xy.odo_last[1];
	xy.ang_inc = (float)(xy.odo_inc[0] - xy.odo_inc[1]) * ODO_Rate / LVDAIJULI;
	xy.angle += xy.ang_inc;
	//float odo_avg = 0.5 * (xy.odo_now[0] + xy.odo_now[1]);
	//xy.position[0] = arm_cos_f32(xy.angle) * odo_avg * ODO_Rate;
	//xy.position[1] = arm_sin_f32(xy.angle) * odo_avg * ODO_Rate;
	float odo_avg = 0.5 * (xy.odo_inc[0] + xy.odo_inc[1]);
	xy.position[0] += arm_cos_f32(xy.angle) * odo_avg * ODO_Rate;
	xy.position[1] += arm_sin_f32(xy.angle) * odo_avg * ODO_Rate;
	xy.position[2] = angZ;
	xy.degree = fmod(xy.angle * 57.2957795, 360);
	if (xy.degree > 180)	xy.degree -= 360;
}
//--------------------------------------------------

void deadZone_func(int32_t* input)
{
	if (*input >= -DEADZONE_BOUND && *input <= DEADZONE_BOUND)	*input = 0;
	else if (*input > DEADZONE_BOUND)	*input -= 340;
	else if (*input < -DEADZONE_BOUND)	*input += 340;
}

void limitRpm_func(int32_t* input)
{
	if (*input >= SPEED_MAX)	*input = SPEED_MAX;
	else if (*input <= SPEED_MIN)	*input = SPEED_MIN;
}

void limit_deadZone(int32_t* speedBuf)
{
	deadZone_func(speedBuf);
	deadZone_func(speedBuf + 1);
}

void limit_rpm(int32_t* rpmBuf)
{
	limitRpm_func(rpmBuf);
	limitRpm_func(rpmBuf + 1);
}

//---------将通道值转换为电机转速------------------------------
void chValToRpm(uint16_t* chval)
{
	int32_t tmpBuf[2];
	tmpBuf[0] = DEADZONE_SLOPE * (*chval - 1000);
	tmpBuf[1] = DEADZONE_SLOPE * (*(chval + 1) - 1000);
	limit_deadZone(tmpBuf);
	if (*chval >= 1000) {
		rpmBuf[0] = tmpBuf[0] + tmpBuf[1];
		rpmBuf[1] = tmpBuf[0] - tmpBuf[1];
	} else {
		rpmBuf[0] = tmpBuf[0] - tmpBuf[1];
		rpmBuf[1] = tmpBuf[0] + tmpBuf[1];
	}
	limit_rpm(rpmBuf);
}
//--------------------------------------------------------------

//------------陀螺仪角度转换----------------------------------------
void angle_func(float* angle)
{
	if (*angle >= 180)		*angle = 360 - *angle;
	else								*angle = -*angle;
}
//----------------------------------------------------
	uint8_t  deng=1;

void RunLedTask(void *argument)
{
	static float n=0; 
	//------遥控器变量---------------------------------------
	uint8_t ykq_tmp[70];
	uint8_t ykq[35] = {'a', 'b'};									//遥控器数据
	uint8_t parity = 0;														//校验位
	uint8_t conFlag = 0x04;												//连接标志位
	uint16_t ch5_val = CH_MID;
	static uint16_t chVal[2] = {CH_MID, CH_MID};
	uint8_t disconNum = 0;
	#if UART1_SEND_ENABLE
	unsigned char sendBuf[110];
	#endif
	//---------------------------------------------------------
	
	//-------------陀螺仪数据-------------------------------------
	uint8_t sAngBuf[11] = {0, 0};
	uint8_t mAngBuf[11] = {0, 0};
	float sAngle_z = 0.0f;
	uint8_t  angbuf3[66]={'a', 'b'};	//陀螺仪串口数据
	uint8_t  angbuf4[44]={'a', 'b'};
	//--------------------------------------------------------
	uint8_t  LEDCNT=0;
	uint16_t djprtQJ=0;
	uint16_t djprtHT=0;

	uint16_t goalCURRENT=0;//预设的舵机停止电流

	uint8_t  safetime=100;
	uint8_t  safecnt=0;	
	uint8_t  bi=0;

// UartDmaSend(2,uartbuf,50);
	GpioReset(GreenLed);//亮
	osDelay(50);

	for(;;)
	{
		memcpy(ykq_tmp, rcvBuf, UART_BUF_SIZE);
		comm_can_get_odo(CTRL_ID_1);
		osDelay(1);
		comm_can_get_odo(CTRL_ID_2);
		
		
	
				
		
//---------遥控器控制------------------------------------------------
		parity = 0x00;
		//遥控器数据获取
		for (bi = 0; bi < 36; bi++)
		{		
			if (ykq_tmp[bi] == 0x0F)
			{
				memcpy(ykq ,ykq_tmp + bi, 35);
				for (bi = 1; bi < 34; bi++) 
				{
					parity ^= ykq[bi];
				}
				break;
			}
		}
		
		//判断校验位、获取遥控器通道值
		if (parity == ykq[34])
		{
			conFlag = ykq[33];					
			chVal[1] = (ykq[1] << 8) + ykq[2];	
			chVal[0] = 2000 - (ykq[3] << 8) - ykq[4];
			ch5_val = (ykq[9] << 8) + ykq[10];
			if (ch5_val == CH_MAX)	chVal[1] = CH_MID;
			else if (ch5_val == CH_MIN)	chVal[0] = CH_MID;
			//数值异常处理
			if((chVal[0] < CH_MIN) || (chVal[1] < CH_MIN) || (chVal[0] > CH_MAX) || (chVal[1] > CH_MAX)) {
				conFlag = 0x04;
			}
		}	
		//memset(rcvBuf, 0, sizeof rcvBuf);
		//memset(ykq, 0, sizeof ykq);
	
		real_odo1 = odoBuf[0] * ODO_Rate;
		real_odo2 = odoBuf[1] * ODO_Rate;
		
		solve_kinematic_model();
		
		#if UART1_SEND_ENABLE
		sprintf(sendBuf, "odo1:%010d,%010d,%07.3fm;odo2:%010d,%010d,%07.3fm;pos:%07.3f,%07.3f,%07.2f\n",
						odoBuf[0], odoAbsBuf[0], real_odo1,
						odoBuf[1], odoAbsBuf[1], real_odo2,
						xy.position[0], xy.position[1], xy.degree);
		UartDmaSend(1, sendBuf, 100);
		#endif
		
		//履带车控制
		if (!conFlag)
		{
			chValToRpm(chVal);
			comm_can_set_rpm(CTRL_ID_1, rpmBuf[0]);
			osDelay(1);
			comm_can_set_rpm(CTRL_ID_2, rpmBuf[1]);
			disconNum = 0;
		}
		else if (NowC[2] < 10)
		{
			chValToRpm(NowC);
			comm_can_set_rpm(CTRL_ID_1, rpmBuf[0]);
			osDelay(1);
			comm_can_set_rpm(CTRL_ID_2, rpmBuf[1]);
			disconNum = 0;
		}
		else {
			//丢失信号处理
			if (disconNum <= 3) {
				comm_can_set_rpm(CTRL_ID_1, 0);
				osDelay(1);
				comm_can_set_rpm(CTRL_ID_2, 0);
				++ disconNum;
			}
			
		  ///////////////////////////////////////////////////	
//		NowC[2];//收到指令了置0，其他时间在网络发送任务里面加，加了10次还没收到，代表UDP中断，之后一直保持10.
//		NowC[0];//遥控器上下通道值	
//		NowC[1];//遥控器左右通道值			
		  ////////////////////////////////////////////////////	
		 }


		 
		 
		
//-----------------------------------------------------------		
		
		//-------三轴陀螺仪 USART3----------------------------------------------------
		parity = 0xA8;
		memcpy(angbuf3, rcvBuf3, UART_BUF_SIZE3);
		for (bi = 0; bi < 57; bi ++)
		{
			if (angbuf3[bi] == 0x55 && angbuf3[bi + 1] == 0x53)
			{
				memcpy(mAngBuf, angbuf3 + bi, 11);
				break;
			}
		}
		for (bi = 2; bi < 10; bi ++)	parity += mAngBuf[bi];
		if (parity == mAngBuf[10])
		{
			angX = (float)((mAngBuf[3] << 8) | mAngBuf[2]) / 32768.0f * 180.0f;
			angY = (float)((mAngBuf[5] << 8) | mAngBuf[4]) / 32768.0f * 180.0f;
			angZ = (float)((mAngBuf[7] << 8) | mAngBuf[6]) / 32768.0f * 180.0f;
			angle_func(&angZ);
		}
		//--------------------------------------------------------------------------
		
		/*
		//---------单轴陀螺仪 UART4------------------------------------------------------
		parity = 0xA8;
		memcpy(angbuf4, rcvBuf4, UART_BUF_SIZE4);
		for (bi = 0; bi < 35; bi ++)
		{
			if (angbuf4[bi] == 0x55 && angbuf4[bi + 1] == 0x53)
			{
				memcpy(sAngBuf, angbuf4 + bi, 11); 
				break;
			}
		}
		parity += sAngBuf[6] + sAngBuf[7] +  0x108;
		if (parity == sAngBuf[10])
		{
			angZZ = (float)((sAngBuf[7] << 8) | sAngBuf[6]) / 32768.0f * 180.0f;
			angle_func(&angZZ);
		}
		//------------------------------------------------------------------------------
		*/
				
/////*-*-*-*-*-*+*+*材料挤出电流控制，舵机保护********************************////		
ADC_ConvertedValueLocal[0] =(float) ADC_ConvertedValue[0]/4096*(float)3.3;
djCURRENT=ADC_ConvertedValueLocal[0]*2481;////实际电流，单位ma 这个2481是用电阻标定算出的一个系数。
	
		//ADC_ConvertedValueLocal[1] =(float) ADC_ConvertedValue[1]/4096*(float)3.3;
//ADC_ConvertedValueLocal[2] =(float) ADC_ConvertedValue[2]/4096*(float)3.3;	
//ADC_ConvertedValueLocal[3] =(float) ADC_ConvertedValue[3]/4096*(float)3.3;	
//ADC_ConvertedValueLocal[4] =(float) ADC_ConvertedValue[4]/4096*(float)3.3;	
//ADC_ConvertedValueLocal[5] =(float) ADC_ConvertedValue[5]/4096*(float)3.3;
//ADC_ConvertedValueLocal[6] =(float) ADC_ConvertedValue[6]/4096*(float)3.3;
//ADC_ConvertedValueLocal[7] =(float) ADC_ConvertedValue[7]/4096*(float)3.3;	
//ADC_ConvertedValueLocal[8] =(float) ADC_ConvertedValue[8]/4096*(float)3.3;	
//ADC_ConvertedValueLocal[9] =(float) ADC_ConvertedValue[9]/4096*(float)3.3;	
if((TIM3->CCR3)>1600)//保护电流和PWM波关系，启动800对应1600,fei非线性
{
    if(TIM3->CCR3==1650){goalCURRENT=1000;}
	else if((TIM3->CCR3>1650) && (TIM3->CCR3<=1700) ){goalCURRENT=1100;}
  else if((TIM3->CCR3>1700) && (TIM3->CCR3<=1800) ){goalCURRENT=1300;}
	else if((TIM3->CCR3>1800) && (TIM3->CCR3<=1900) ){goalCURRENT=1500;}
	else{goalCURRENT=1500;}
	
	//1600才转 1500以上回退 1500以下挤出
	//1650   857
	//1700 1030	
	//1800   1381
		  	
        if(goalCURRENT<=600){goalCURRENT=600;}//限制最大挤出力，
   		if(goalCURRENT>=1700){goalCURRENT=1700;}//限制最大回退力		  
	   if(djCURRENT-goalCURRENT>500)
		   { 
	     TIM3->CCR3=1500; 
		   }//本次到达目标力电流	 	
	}

else if(TIM3->CCR3<1400)//挤出
    {		

  if(djCURRENT>=3000){ TIM3->CCR3=1500; }//保护舵机			
//1000 1700
	
	}		

		
		
		
/////*-*-*-*-*-*+*+****************************************///////////////		
		if(safeFlag==1)
		{
		safetime=100;
		}
		else if	(safeFlag==0)
		{
		safetime++;
		}
		
      if(safetime>=140)//2秒时间
	  {
		safetime=100;
//		TIM4->CCR1=900;//关闭所有作业工具,20240801GUANBI关闭
//	//	TIM4->CCR2=1500;
//		TIM4->CCR3=1500;
//		TIM4->CCR4=900;
////	AQ	1TIM3->CCR3=1500;	    	  
	  }
	  
	if(safecnt==20)//1秒时间置零，如果没有心跳指令过来，就一直是0
      {
	  safecnt=0;	  
	  safeFlag=0;		  
	  }		
/////***********************************************///////////////////////
	if(LEDCNT==19)
		{	
		 GpioReset(GreenLed);//亮
		}	//					  
	else if(LEDCNT==20)
		{
		LEDCNT=0;   
		GpioSet(GreenLed);	//mie灭
	//UartDmaSend(2,uartbuf,50);    
		}			
		

		/*
	 if (NowT[0] == 3)   //力传感器清零
	 {
		 SetForceZero();
		 NowT[0] = '2';
	 }
		*/
		 
		
////**************************************************///////////////	
		
//GpioSet(GreenLed);	//mie灭
//nimatic(goalx,goaly,goalz,&arm2022);		
//GpioReset(GreenLed);//亮
//nimatic(goalx,goaly,goalz,&arm2022);			
//GpioSet(GreenLed);	//mie灭
		
		safecnt++;
		LEDCNT++;
		osDelay(49);	
	}
}


