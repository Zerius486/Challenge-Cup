#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "sys.h"

#include "JointTask.h"
#include "jlCAN.h"
#include <string.h>

#include <stdio.h>
#include <stdlib.h>

#include "kinematics.h"
//#include "math.h" 
#include "usartdma.h"
#include  "arm_math.h"




#include "gpio.h"
#include "JLGpio.h"

#include "api.h"

const float32_t d1=223.5f; 
//const float32_t  a2=413.1f;//
//cnst float32_t  a3=212.03;	//机械臂原长
//const float32_t  a3=370.1;	//	装夹爪后长度
//const float32_t  a3=410.1f;	//装笔后长度	
//const float32_t  a3=295.1f;	//装侧笔后长度	

const float32_t theta1 = 0.0461f;
const float32_t theta2 = 0.0461f;
const float32_t theta3 = 0.104f; //末端工具偏转角度


const float32_t I1xx= 0.0033364f;
const float32_t I1xy= 0.0f;
const float32_t I1xz= 0.0f;
const float32_t I1yy= 0.0026844f;
const float32_t I1yz= 0.00011538f;
const float32_t I1zz= 0.0025812f;

const float32_t I2xx= 0.0029956f;
const float32_t I2xy= 0.0012416f;
const float32_t I2xz= -0.00039946f;
const float32_t I2yy= 0.0163f;
const float32_t I2yz= -0.0001901f;
const float32_t I2zz= 0.016422f;

const float32_t I3xx= 0.0029059f;
const float32_t I3xy= -0.00021301f;
const float32_t I3xz= -0.00031948f;
const float32_t I3yy= 0.0031557f;
const float32_t I3yz= -0.00012645f;
const float32_t I3zz= 0.0029024f;

const float32_t  m1=2.0972f;
const float32_t  m2 =  2.7077f;
//const float32_t  m3=   3.0144f;	//夹爪重1.1kg
const float32_t  m3=   1.9144f;	//无夹爪，只有笔

const float32_t  lm2=  0.2719f;		
const float32_t  lm3=  0.1876f;	//

const float32_t  g  =  9.7949f;	//南京9.7949 郑州9.7966 上海9.7964 广州9.7833
const float32_t  l0 =  0.139f;
const float32_t  l1 =  0.0845f;
const float32_t  l2 =  0.4131f;
const float32_t  l3 =  0.3702f;
const float32_t  m1x=   0.0f;
const float32_t  m1y=   -0.013697f;
const float32_t  m1z=   0.077671f;
const float32_t FiveMsDate[1]=   {0.005f};


void JointInit(unsigned char _dat);
void JointV5Init(unsigned char _dat);
void PosSend(unsigned char MotorID,unsigned int POSCNT);
void PosSendV5 (unsigned char MotorID,unsigned int POSCNT);
void TorSend(unsigned char MotorID,int16_t current);
void ReadDianYa(unsigned char MotorID);
void ReadForce();

void TPDOA(void);
void TPDOB(void);
void TPDOC(void);
void TPDOD(void);
void TPDOSTART(void);
void TPDOSTOP(void);
void sync(void);
void ALLSTOP(void);

double kalmanFilter(double inputz);




unsigned char JointCmd_ClearErr[8] 	= {0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00};		//清除错误
unsigned char JointCmd_PP[8] 		= {0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00};		//设定为位置模式，第五个字节
unsigned char JointCmd_PV[8] 		= {0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00};		//设定为速度模式
unsigned char JointCmd_TQ[8] 		= {0x2F,0x60,0x60,0x00,0x04,0x00,0x00,0x00};		//设定为力矩模式
unsigned char JointCmd_CB[8] 		= {0x2F,0x60,0x60,0x00,0x07,0x00,0x00,0x00};		//设定为插补位置模式

//unsigned char JointCmd_SetAcc[8]  	= {0x23,0x83,0x60,0x00,0x10,0x27,0x00,0x00};		//设置加速度10000=0x2710
//unsigned char JointCmd_SetDcc[8]  	= {0x23,0x84,0x60,0x00,0x10,0x27,0x00,0x00};		//设置减速度
//unsigned char JointCmd_SetPPSpd[8] 	= {0x23,0x81,0x60,0x00,0x10,0x27,0x00,0x00};		//设置位置模式下的速度

unsigned char JointCmd_SetAcc[8]  	= {0x23,0x83,0x60,0x00,0x20,0x4e,0x00,0x00};		//设置加速度20000=0x4e20
unsigned char JointCmd_SetDcc[8]   	= {0x23,0x84,0x60,0x00,0x20,0x4e,0x00,0x00};		//设置减速度
unsigned char JointCmd_SetPPSpd[8] 	= {0x23,0x81,0x60,0x00,0x20,0x4e,0x00,0x00};		//设置位置模式下的速度

//unsigned char JointCmd_SetAcc[8] 	  = {0x23,0x83,0x60,0x00,0x50,0xc3,0x00,0x00};		//设置加速度50000=0xc350
//unsigned char JointCmd_SetDcc[8]  	= {0x23,0x84,0x60,0x00,0x50,0xc3,0x00,0x00};		//设置减速度
//unsigned char JointCmd_SetPPSpd[8] 	= {0x23,0x81,0x60,0x00,0x50,0xc3,0x00,0x00};		//设置位置模式下的速度


unsigned char JointCmd_ReadPos[8] 	= {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00};		//读取当前位置
unsigned char JointCmd_SetPos[8] 	= {0x23,0x7A,0x60,0x00,0x00,0x00,0x00,0x00};		//设置位置
unsigned char JointCmd_SetSpd[8] 	= {0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00};		//设置速度
unsigned char JointCmd_SetCur[8] 	= {0x2B,0x71,0x60,0x00,0x00,0x00,0x00,0x00};		//设置电流

unsigned char JointCmd_MtStop[8] 	= {0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};		//电机停止
unsigned char JointCmd_MtSevr[8] 	= {0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00};		//伺服开始//伺服准备
unsigned char JointCmd_MtEn[8] 		= {0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00};		//电机使能
unsigned char JointCmd_MtStart[8]	= {0x2B,0x40,0x60,0x00,0x1F,0x00,0x00,0x00};		//电机开始运行

unsigned char JointCmd_bit40[8] = {0x2B,0x40,0x60,0x00,0x2F,0x00,0x00,0x00};		//bit4置0，
unsigned char JointCmd_bit41[8]	= {0x2B,0x40,0x60,0x00,0x3F,0x00,0x00,0x00};		//bit4置1，0变成1，电机开始运行



unsigned char TPDOSynClose[8]	= {0x23,0x05,0x10,0x00,0x80,0x00,0x00,0x00};//关闭同步发生器（1005h写00）
unsigned char TPDOSynTime[8]	= {0x23,0x06,0x10,0x00,0x10,0x27,0x00,0x00};//自动发送同步帧时间5000us=0x1388/   2000=0x07d0、10ms=10000=0x2710

unsigned char TPDOIdA[8]	= {0x23,0x00,0x18,0x01,0x8A,0x01,0x00,0x80};//返回帧的id号 0x18A
unsigned char TPDOIdB[8]	= {0x23,0x00,0x18,0x01,0x8B,0x01,0x00,0x80};//返回帧的id号 0x18B
unsigned char TPDOIdC[8]	= {0x23,0x00,0x18,0x01,0x8C,0x01,0x00,0x80};//返回帧的id号 0x18C
unsigned char TPDOIdD[8]	= {0x23,0x00,0x18,0x01,0x8D,0x01,0x00,0x80};//返回帧的id号 0x18D
unsigned char TPDOkind[8]	= {0x2F,0x00,0x18,0x02,0x01,0x00,0x00,0x00};//设定TPDO1传输类型为1（1个同步帧循环传输）
unsigned char TPDOft[8]	  = {0x2B,0x00,0x18,0x03,0x14,0x00 ,0x00,0x00};//设定TPDO1传输禁止时间为2ms，单位：100us


unsigned char TPDOdsq[8]	= {0x2B,0x00,0x18,0x05,0x00,0x00,0x00,0x00};//设定TPDO1传输定时器（仅针对异步传输）

unsigned char TPDOclr[8]	= {0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00};//TPDO1子索引清零
unsigned char TPDOpos[8]	= {0x23,0x00,0x1A,0x01,0x20,0x00,0x64,0x60};//1A00h-01h映射60640020h（实际位置）
unsigned char TPDOspd[8]	= {0x23,0x00,0x1A,0x02,0x20,0x00,0x6C,0x60};//1A00h-02h映射606C0020h（实际速度）

unsigned char TPDOcur[8]    = {0x23,0x00,0x1A,0x02,0x10,0x00,0x78,0x60};//1A00h-02h映射606C0020h（实际电流）      

unsigned char TPDONum[8]	= {0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00};//设定TPDO1子索引数目为2
unsigned char TPDOAON[8]    = {0x23,0x00,0x18,0x01,0x8A,0x01,0x00,0x00};//激活TPDO idA
unsigned char TPDOBON[8]	    = {0x23,0x00,0x18,0x01,0x8B,0x01,0x00,0x00};//激活TPDO idB
unsigned char TPDOCON[8]    = {0x23,0x00,0x18,0x01,0x8C,0x01,0x00,0x00};//激活TPDO idC
unsigned char TPDODON[8]    = {0x23,0x00,0x18,0x01,0x8D,0x01,0x00,0x00};//激活TPDO idD

unsigned char TPDOstart[8]    = {0x23,0x05,0x10,0x00,0x80,0x00,0x00,0x40};//开始自动发送同步帧
unsigned char TPDOstop[8]    = {0x23,0x05,0x10,0x00,0x80,0x00,0x00,0x00};//关闭自动发送同步帧








// 全局遍历 卡尔曼滤波参数
double x = 0.0;  // 初始状态（初始力传感器值）
double P = 1.0;  // 初始协方差
double Q = 0.01;  // 过程噪声协方差,可调试参数
double R = 5.0;  // 测量噪声协方差，可调试参数
double kalmanFilter(double z);
double z[15] = {-10.0, -10.0, -10.0, -10.0, -9.9, -9.9, -9.9, -10, -10, -10, -10, -10, -9.9, -9.9, -9.9};
/////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char JointInitFlag = 0;
unsigned char safeFlag=0;
unsigned char huituiFlag=0;



unsigned int NowPosition[16] = {0,},NowVelocity[16] = {0,};

int32_t NowP[16] = {0,},NowV[16] = {0,},NowT[16] = {0,};

int16_t NowC[16] = {1000,1000,10,0,0,0,0,0,0,0,0,0,0,0,0,0};

float angA,angB,angC,angD;

unsigned int AimPosition = 0,AimVelocity = 0,AimCurrent = 0;

float perr=0,verr=0;

float perrB=0,verrB=0;	
float perrC=0,verrC=0;	
float perrD=0,verrD=0;	

int16_t  _tempcurrentB = 0;
int16_t  _tempcurrentC = 0;
int16_t  _tempcurrentD = 0;
int16_t  _tempcurrent = 0;	//电流比列，-1000到1000，代表占空比-100到100，必须用16位有符号整数
float32_t PidData[3]=   {0.0f,0.0f,0.0f};

float kp,kd,ki,Tourque;

float kpB,kdB;
float kpC,kdC;
float kpD,kdD;

endposdef arm2022;
eulAngle eul;

int32_t golP,golV;
int32_t golPB,golVB;
int32_t golPC,golVC;
int32_t golPD,golVD;


int8_t ContMode=0;
int8_t lastMode=0;

int8_t GuiJi=0;
int8_t huayuan=-1;
int8_t NewComeIn=0;
float goalx=100,goaly=0,goalz=450;

float Fkp=0.5,Fki=0,Fkd=0.05;
float baseX=350,baseY=0,baseZ=400;

float rad=0;
u16_t radi=0;
///////////////////////////////////////////// 

/////////////////////////////////////////////////
int8_t Mxstate[11]={10,10,10,10,10,10,10,10,10,10,10};
int16_t  OutCut[3]=   {0.0f,0.0f,0.0f};

//float32_t q1,q2,q3;
//float32_t DOB1=0.1f;

float32_t	eX=0,eY=0,eZ=0; 

//float32_t	rX,rY,rZ;
//float32_t  cs1,cs2,cs3,si1,si2,si3;
float32_t	FZErrLast=0.0,FZNow=0.0,FZErr=0.0,FZgoal=0.0,FZErrSum=0.0;

float32_t  forceM;


float32_t	PID_cal=0.0;

uint16_t  cnn=0;
//const uint16_t fasongbuf=405;
//unsigned char TxBuf1[fasongbuf]={0,};	
//unsigned char TxBuf2[fasongbuf]={0,};	

void JointTask(void *argument)
{
	
		CAN_TxHeaderTypeDef ddTxHeader;
	unsigned char ddTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ddTxMailBox = CAN_TX_MAILBOX0;
	
	ddTxHeader.IDE = CAN_ID_STD;
	ddTxHeader.RTR = CAN_RTR_DATA;
	ddTxHeader.StdId = 0x6A;
	ddTxHeader.DLC = 8;
	
	
	
unsigned int  _temp = 0;	
uint16_t  TIMEsecond = 0;
unsigned char huancunFlag=0;	
unsigned char tempBuf[40]={0,};	
uint8_t i=0;		
float yuanx=-60;

uint8_t stop=0;


	osDelay(8000);//关节需要上电后等待8秒，才能向其发送数据。	
	TPDOSTOP();	
	TPDOA();	
  osDelay(5);	
	TPDOB();	
    osDelay(5);	
	TPDOC();
    osDelay(5);	
	TPDOD();
    osDelay(5);	
    TPDOSTART();//为了获取当前位置
    osDelay(500);	
	TPDOSTOP();
 osDelay(500);	


JointV5Init(0x1C);	
 osDelay(500);	
  JointInit(0x1A);//四个关节初始化，位置模式，在当前位置	
	JointInit(0x1B);	
	
	JointInit(0x1D);	
    osDelay(50);
  
  nimatic(355,0,455,&arm2022); 
	_temp=arm2022.jd1;
PosSend(0X0D,_temp);	
	_temp=arm2022.jd2;
PosSendV5(0X0C,_temp);	
	_temp=arm2022.jd3;
PosSend(0X0B,_temp);
osDelay(3000);      //运动到初始位置
 
 nimatic(320,0,400,&arm2022); 
	_temp=arm2022.jd1;
PosSend(0X0D,_temp);	
	_temp=arm2022.jd2;
PosSendV5(0X0C,_temp);	
	_temp=arm2022.jd3;
PosSend(0X0B,_temp);
osDelay(1000);      //运动到初始位置
//JointInit(0x3A);//力距模式
//JointInit(0x3B);//力距模式
//JointInit(0x3C);//力距模式
//JointInit(0x3D);//力距模式

	TPDOA();	
  osDelay(5);	
	TPDOB();	
    osDelay(5);	
	TPDOC();
    osDelay(5);	
	TPDOD();
    osDelay(5);	
		
    TPDOSTART();
    osDelay(5);	

kp=0.5;//力控置pid参数
ki=0;
kd=0.05;

NowT[15]=0;//这是目标力初始值，之后新值通过udp传入
	for(;;)
	{
		
		
				ReadForce();

		if (ContMode == 1 || ContMode == 2)	
		{
			stop=0;	
			//末端位置限位
			/*
			if(goalx>600){goalx=600;};
			if(goalx<-200){goalx=-200;};
		
      if(goaly>300){goaly=300;};
			if(goaly<-300){goaly=-300;};
		
			if(goalz>600){goalz=600;};
			if(goalz<-300){goalz=-300;};							
			*/
			
			nimatic(goalx,goaly,goalz,&arm2022); 
			//tr2eul(&arm2022, &eul);//末端点的欧拉角
			
			PosSend(0x0B,arm2022.jd3); 
			osDelay(1);		
			PosSendV5(0x0C,arm2022.jd2); 
			osDelay(1);		
			PosSend(0x0D,arm2022.jd1); 
			osDelay(1);	 
		}
		else if (ContMode == 3)  //关节角度控制
    {
	   	stop=0;
			//ret->jd1 = -83443.02679286822 * theta1 + 231000;
			//ret->jd2 = -83443.02679286822 * theta2 + 272430; 
			//ret->jd3 = -83443.02679286822 * theta3 + 365722; 
			//ret->jd4 = 0; 
			arm2022.jd1 = -83443.02679286822 * angA + spe1;  //底座关节
      arm2022.jd2 = -83443.02679286822 * angB + spe2; 
      arm2022.jd3 = -83443.02679286822 * angC + spe3; 
			//arm2022.jd1=-83443.02679286822*angD+362000;
      arm2022.jd4 = -83443.02679286822 * angD + spe4;    			//angle4对应末端旋转关节
	   
			PosSend(0x0A,arm2022.jd4);  //末端关节是A，对应上位机最后数据
			osDelay(1);	
			PosSend(0x0B,arm2022.jd3);
	    osDelay(1);		
			PosSendV5(0x0C,arm2022.jd2);
	    osDelay(1);		
			PosSend(0x0D,arm2022.jd1);
		}
		else if(ContMode == 4  && stop == 0)
    {		
			stop=1;
			ALLSTOP();
				 
			JointV5Init(0x1C);	   
			JointInit(0x1A);//四个关节初始化，位置模式，在当前位置	
			JointInit(0x1B);	
			JointInit(0x1D);	
			osDelay(50);
		}				 
		 
		else if(ContMode == 5)//只获取z方向的力。怼垂直墙面，发送FXOP:10,就是10N的力,FXOP:10,0100,0100,0010,后面是pid参数。
    {
			stop=0;
//		NowT[4] = _temp32;		//f_Y	
//		NowT[5] = _temp32;		//f_Z			
//		NowT[2] = _temp32;		//f_Z_t
//		NowT[3] = _temp32;		//f_x	
			
	
		kp=	Fkp;
		ki=Fki;
    kd=	Fkd;		
			
		FZgoal=	NowT[15];//设定目标值
				if( (FZgoal>100)|| (FZgoal<-100)){FZgoal=0;}	
				
		forceM=	NowT[5];
		forceM=-(forceM/1000);	
		
		if( (FZNow>400)|| (FZNow<-400)){FZNow=888;}						
    FZNow= kalmanFilter(forceM);
		angZZ = FZNow;
	
		
		
		
		FZErr=FZgoal-FZNow;		
		if( (FZErr<0.2)&&  (FZErr>-0.2)){FZErr=0;}	//在这个区间，认为是漂移。
		FZErrSum=FZErrSum+FZErr;//误差累积，积分项目，即i项。
	 if( FZErrSum>= 500 ) { FZErrSum=500;}//积分限幅
   if( FZErrSum<= -500 ) {  FZErrSum=-500;}		
	 
		PID_cal=kp*FZErr + ki*FZErrSum+   kd*(FZErr-FZErrLast);//微分项，即d项，是误差的变化率。		
		FZErrLast=FZErr;//误差传递	 
		goalx=	eX+PID_cal;		
	//	eX ;//当前位置
//		eY=0 ;
//		eZ=400;//差不多水平位置

			//末端位置限位
	
		  if(goalx>650){goalx=650;};//对X进行限位
			if(goalx<300){goalx=300;};
					/*
      if(goaly>300){goaly=300;};
			if(goaly<-300){goaly=-300;};
		
			if(goalz>600){goalz=600;};
			if(goalz<-300){goalz=-300;};							
			*/		
			
			goaly=-100+200*sin(rad);
			rad=rad+0.0005;
			if(rad>=3.1415 )
			{		
			rad=3.1415;
				radi=radi+1;
				if(radi>1000)//10zantin暂停10秒
					{   radi=0;
				     rad=0;
					}
			}
			
			
			
			nimatic(goalx,goaly,470,&arm2022); 
			//tr2eul(&arm2022, &eul);//末端点的欧拉角
			
			PosSend(0x0B,arm2022.jd3); 
			osDelay(1);		
			PosSendV5(0x0C,arm2022.jd2); 
	
			PosSend(0x0D,arm2022.jd1); 
			osDelay(1);	 	
					
		}		
		
		else if(ContMode == 6)//地面磨刷
    {
			stop=0;
					
//		kp=	Fkp;
//		ki=Fki;
//    kd=	Fkd;	
		
			FZgoal=	NowT[15];//设定目标值
				if( (FZgoal>50)|| (FZgoal<-50)){FZgoal=0;}					
		forceM=	NowT[5];
		forceM=-(forceM/1000);	

		if( (FZNow>400)|| (FZNow<-400)){FZNow=888;}						
	
    FZNow= kalmanFilter(forceM);//对z轴力进行卡尔曼滤波
		angZZ = FZNow;

		FZErr=FZgoal-FZNow;	
		if( (FZErr<0.2)  && (FZErr>-0.2)){FZErr=0;}	//在这个区间，认为是漂移。			
		
		FZErrSum=FZErrSum+FZErr;//误差累积，积分项目，即i项。
	 if( FZErrSum>= 500 ) { FZErrSum=500;}//积分限幅
   if( FZErrSum<= -500 ) {  FZErrSum=-500;}		
		PID_cal=kp*FZErr + ki*FZErrSum+   kd*(FZErr-FZErrLast);//微分项，即d项，是误差的变化率。		
		FZErrLast=FZErr;//误差传递	 
		goalz=	eZ-PID_cal;		
	


			//末端位置限位
//		  if(goalx>600){goalx=600;};//对X进行限位
//			if(goalx<100){goalx=100;};			
//      if(goaly>300){goaly=300;};
//			if(goaly<-300){goaly=-300;};
		
			if(goalz>400){goalz=400;};
			if(goalz<-550){goalz=-550;};							
				
			//末端轨迹
			goaly= 200*sin(rad);
			rad=rad+0.001;
			if(rad>=6.283 )
			{		
			rad=6.283;
				radi=radi+1;
				if(radi>1000)//10zantin暂停10秒
					{   radi=0;
				     rad=0;
					}
			}	
			
			
			
			nimatic(510,goaly,goalz,&arm2022); 
			//tr2eul(&arm2022, &eul);//末端点的欧拉角
			
			PosSend(0x0B,arm2022.jd3); 
			osDelay(1);		
			PosSendV5(0x0C,arm2022.jd2); 
		
			PosSend(0x0D,arm2022.jd1); 
			osDelay(1);	 			
			
		}				
else if(ContMode == 7)//
    {	
		stop=0;
		}
		
		
		
    TIMEsecond = TIMEsecond + 1;	
		if(TIMEsecond > 100)//2秒
	  {
	    TIMEsecond=0;
			//ReadDianYa(0x0C);		 
	  }			
		

		osDelay(7);	//整体周期7+3ms//7+3
		
	}		
}

//_dat 高四位决定关节模式,0x10是位置，0x20是速度，0x30是力距，0x40是插补，DI四位是ID号0x0a，0x0b，0x0c，0x0d
void JointV5Init(unsigned char _dat)
{

	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x600|(_dat&0xF);
	xtTxHeader.DLC = 8;

	if((_dat&0xF0) == 0x10)		//PP
	{
		memcpy(uctTxBuf,JointCmd_ReadPos,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(100);
		memcpy(uctTxBuf,JointCmd_SetPos,8);                          
		uctTxBuf[4] =  NowPosition[_dat&0xF]&0xFF;
		uctTxBuf[5] = (NowPosition[_dat&0xF]>>8)&0xFF;
		uctTxBuf[6] = (NowPosition[_dat&0xF]>>16)&0xFF;
		uctTxBuf[7] = (NowPosition[_dat&0xF]>>24)&0xFF;
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//设置要运动到的位置60 7a
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_ClearErr,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	////清除错误
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_PP,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	 //设定为轮廓位置模式60 60=0x01
		osDelay(50);

		memcpy(uctTxBuf,JointCmd_SetAcc,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	 //设定加速度
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_SetDcc,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	 //设定减速度
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_SetPPSpd,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	 //设定速度
		osDelay(50);


		memcpy(uctTxBuf,JointCmd_MtStop,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机停止
		osDelay(500);
		memcpy(uctTxBuf,JointCmd_MtSevr,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//伺服开始//伺服准备
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_MtEn,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机使能
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_MtStart,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机开始运行
		osDelay(50);

		memcpy(uctTxBuf,JointCmd_bit40,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机开始运行
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_bit41,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机开始运行
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_bit40,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机开始运行
		osDelay(50);	
	}	
	
	
	
	
	
	
}





//_dat 高四位决定关节模式,0x10是位置，0x20是速度，0x30是力距，0x40是插补，DI四位是ID号0x0a，0x0b，0x0c，0x0d
void JointInit(unsigned char _dat)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x600|(_dat&0xF);
	xtTxHeader.DLC = 8;
	
	if((_dat&0xF0) == 0x10)		//PP
	{
		memcpy(uctTxBuf,JointCmd_ReadPos,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(100);
		memcpy(uctTxBuf,JointCmd_SetPos,8);                          
		uctTxBuf[4] =  NowPosition[_dat&0xF]&0xFF;
		uctTxBuf[5] = (NowPosition[_dat&0xF]>>8)&0xFF;
		uctTxBuf[6] = (NowPosition[_dat&0xF]>>16)&0xFF;
		uctTxBuf[7] = (NowPosition[_dat&0xF]>>24)&0xFF;
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//设置要运动到的位置60 7a
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_ClearErr,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	////清除错误
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_PP,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	 //设定为轮廓位置模式60 60=0x01
		osDelay(50);
		
			memcpy(uctTxBuf,JointCmd_SetAcc,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	 //设定加速度
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_SetDcc,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	 //设定减速度
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_SetPPSpd,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	 //设定速度
		osDelay(50);	
		
		
		
		memcpy(uctTxBuf,JointCmd_MtStop,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机停止
		osDelay(500);
		memcpy(uctTxBuf,JointCmd_MtSevr,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//伺服开始//伺服准备
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_MtEn,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机使能
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_MtStart,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机开始运行
		osDelay(50);
	}
	else if((_dat&0xF0) == 0x20)	//PV
	{
		memcpy(uctTxBuf,JointCmd_ClearErr,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_PV,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_SetSpd,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_MtStop,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(500);
		memcpy(uctTxBuf,JointCmd_MtSevr,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_MtEn,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
	}
	else if((_dat&0xF0) == 0x30)	//TQ
	{
		memcpy(uctTxBuf,JointCmd_ClearErr,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_TQ,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_SetCur,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_MtStop,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(500);
		memcpy(uctTxBuf,JointCmd_MtSevr,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
		memcpy(uctTxBuf,JointCmd_MtEn,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
		osDelay(50);
	}

	
	
}

void PosSend(unsigned char MotorID,unsigned int POSCNT)
{	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
		unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;	
     
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x600|(MotorID&0xF);
	xtTxHeader.DLC = 8;
	 
		uctTxBuf[0] = 0x23;
		uctTxBuf[1] = 0x7A;
		uctTxBuf[2] = 0x60;
		uctTxBuf[4] = POSCNT & 0xFF;
		uctTxBuf[5] = (POSCNT>>8)&0xFF;
		uctTxBuf[6] = (POSCNT>>16)&0xFF;
		uctTxBuf[7] = (POSCNT>>24)&0xFF;//这4个字节是电机位置，由低到高，这里使用的19位编码器，
      HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);
}
void PosSendV5(unsigned char MotorID,unsigned int POSCNT)
{	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
		unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;	
     
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x600|(MotorID&0xF);
	xtTxHeader.DLC = 8;
	 
		uctTxBuf[0] = 0x23;
		uctTxBuf[1] = 0x7A;
		uctTxBuf[2] = 0x60;
		uctTxBuf[4] = POSCNT & 0xFF;
		uctTxBuf[5] = (POSCNT>>8)&0xFF;
		uctTxBuf[6] = (POSCNT>>16)&0xFF;
		uctTxBuf[7] = (POSCNT>>24)&0xFF;//这4个字节是电机位置，由低到高，这里使用的19位编码器，
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);
		
		memcpy(uctTxBuf,JointCmd_bit41,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机开始运行
		osDelay(1);
		memcpy(uctTxBuf,JointCmd_bit40,8);
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	//电机开始运行
	
}


void TorSend(unsigned char MotorID,int16_t current)
{
   CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x600|(MotorID&0xF);
	xtTxHeader.DLC = 8;
	
  		uctTxBuf[0] = 0x2B;
		uctTxBuf[1] = 0x71;
		uctTxBuf[2] = 0x60;
		uctTxBuf[4] = current & 0xFF;
		uctTxBuf[5] = (current>>8)&0xFF;//这4个字节是电机电流，
		uctTxBuf[6] = 0;
		uctTxBuf[7] = 0;
HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);
		
}

void ReadDianYa(unsigned char MotorID)
{
   CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x064d;
	xtTxHeader.DLC = 2;	
	
  		uctTxBuf[0] = 00;
		uctTxBuf[1] = 0x24;
HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
	
}

void ReadForce()
{
  CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	xtTxHeader.IDE = CAN_ID_EXT;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.ExtId = 0x00020110;
	xtTxHeader.DLC = 4;	
  uctTxBuf[0] = 0x03;
	uctTxBuf[1] = 0x00;
	uctTxBuf[2] = 0xB0;
	uctTxBuf[3] = 0x0C;	
	HAL_CAN_AddTxMessage(&hcan1, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
}

void SetForceZero()
{
  CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[4] = {0x10,0x00,0xAC,0x00};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	xtTxHeader.IDE = CAN_ID_EXT;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.ExtId = 0x00020110;
	xtTxHeader.DLC = 4;	
	HAL_CAN_AddTxMessage(&hcan1, &xtTxHeader, uctTxBuf, &ultTxMailBox);
}



void TPDOA(void)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x00;
	xtTxHeader.DLC = 2;
	uctTxBuf[0]=0x82;
	uctTxBuf[1]=0x0A;	//复位节点
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);	
	
	xtTxHeader.StdId = 0x60A;		
	xtTxHeader.DLC = 8;
	memcpy(uctTxBuf,TPDOSynClose,8);//关闭同步发生器
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);	

	memcpy(uctTxBuf,TPDOSynTime,8);//自动发送同步帧时间
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);

	memcpy(uctTxBuf,TPDOIdA,8);//设置返回帧的id号 0x18A
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);

	memcpy(uctTxBuf,TPDOkind,8);//设定TPDO1传输类型为1
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);

	memcpy(uctTxBuf,TPDOft,8);//设定TPDO1传输禁止时间为2ms
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);

	memcpy(uctTxBuf,TPDOdsq,8);//设定TPDO1传输定时器
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);	
	memcpy(uctTxBuf,TPDOclr,8);//TPDO1子索引清零
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);	
	memcpy(uctTxBuf,TPDOpos,8);//1A00h-01h映射60640020h（实际位置）
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);	
	
//	memcpy(uctTxBuf,TPDOspd,8);//1A00h-02h映射606C0020h（实际速度）
//    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
//    osDelay(5);	
	
	
	memcpy(uctTxBuf,TPDOcur,8);//1A00h-02h映射606C0020h（实际电流）
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);		
	
	

	memcpy(uctTxBuf,TPDONum,8);//设定TPDO1子索引数目为2
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);
	
	memcpy(uctTxBuf,TPDOAON,8);//激活TPDO idA
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);	
	

	xtTxHeader.StdId = 0x00;
	xtTxHeader.DLC = 2;
	uctTxBuf[0]=0x01;
	uctTxBuf[1]=0x0A;	//启动节点
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(5);			
}

void TPDOB(void)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x00;
	xtTxHeader.DLC = 2;
	uctTxBuf[0]=0x82;
	uctTxBuf[1]=0x0B;	//复位节点
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	
	xtTxHeader.StdId = 0x60B;		
	xtTxHeader.DLC = 8;
	memcpy(uctTxBuf,TPDOSynClose,8);//关闭同步发生器
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	

	memcpy(uctTxBuf,TPDOSynTime,8);//自动发送同步帧时间
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOIdB,8);//设置返回帧的id号 0x18A
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOkind,8);//设定TPDO1传输类型为1
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOft,8);//设定TPDO1传输禁止时间为2ms
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOdsq,8);//设定TPDO1传输定时器
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	memcpy(uctTxBuf,TPDOclr,8);//TPDO1子索引清零
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	memcpy(uctTxBuf,TPDOpos,8);//1A00h-01h映射60640020h（实际位置）
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	
//	memcpy(uctTxBuf,TPDOspd,8);//1A00h-02h映射606C0020h（实际速度）
//    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
//    osDelay(2);	
	memcpy(uctTxBuf,TPDOcur,8);//1A00h-02h映射606C0020h（实际电流）
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	




	memcpy(uctTxBuf,TPDONum,8);//设定TPDO1子索引数目为2
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
	
	memcpy(uctTxBuf,TPDOBON,8);//激活TPDO idB
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	

	xtTxHeader.StdId = 0x00;
	xtTxHeader.DLC = 2;
	uctTxBuf[0]=0x01;
	uctTxBuf[1]=0x0B;	//启动节点
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
}
void TPDOC(void)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x00;
	xtTxHeader.DLC = 2;
	uctTxBuf[0]=0x82;
	uctTxBuf[1]=0x0C;	//复位节点
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	
	xtTxHeader.StdId = 0x60C;		
	xtTxHeader.DLC = 8;
	memcpy(uctTxBuf,TPDOSynClose,8);//关闭同步发生a
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	

	memcpy(uctTxBuf,TPDOSynTime,8);//自动发送同步帧时间
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOIdC,8);//设置返回帧的id号 0x18A
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOkind,8);//设定TPDO1传输类型为1
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOft,8);//设定TPDO1传输禁止时间为2ms
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOdsq,8);//设定TPDO1传输定时器
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	memcpy(uctTxBuf,TPDOclr,8);//TPDO1子索引清零
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	memcpy(uctTxBuf,TPDOpos,8);//1A00h-01h映射60640020h（实际位置）
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	
//	memcpy(uctTxBuf,TPDOspd,8);//1A00h-02h映射606C0020h（实际速度）
//    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
//    osDelay(2);	
	memcpy(uctTxBuf,TPDOcur,8);//1A00h-02h映射606C0020h（实际电流）
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	



	memcpy(uctTxBuf,TPDONum,8);//设定TPDO1子索引数目为2
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
	
	memcpy(uctTxBuf,TPDOCON,8);//激活TPDO idC
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	

	xtTxHeader.StdId = 0x00;
	xtTxHeader.DLC = 2;
	uctTxBuf[0]=0x01;
	uctTxBuf[1]=0x0C;	//启动节点
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
}
void TPDOD(void)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x00;
	xtTxHeader.DLC = 2;
	uctTxBuf[0]=0x82;
	uctTxBuf[1]=0x0D;	//复位节点
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	
	xtTxHeader.StdId = 0x60D;		
	xtTxHeader.DLC = 8;
	memcpy(uctTxBuf,TPDOSynClose,8);//关闭同步发生器
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	

	memcpy(uctTxBuf,TPDOSynTime,8);//自动发送同步帧时间
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOIdD,8);//设置返回帧的id号 0x18D
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOkind,8);//设定TPDO1传输类型为1
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOft,8);//设定TPDO1传输禁止时间为2ms
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);

	memcpy(uctTxBuf,TPDOdsq,8);//设定TPDO1传输定时器
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	memcpy(uctTxBuf,TPDOclr,8);//TPDO1子索引清零
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	memcpy(uctTxBuf,TPDOpos,8);//1A00h-01h映射60640020h（实际位置）
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	

//	memcpy(uctTxBuf,TPDOspd,8);//1A00h-02h映射606C0020h（实际速度）
//    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
//    osDelay(2);	

	memcpy(uctTxBuf,TPDOcur,8);//1A00h-02h映射606C0020h（实际电流）
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	
	memcpy(uctTxBuf,TPDONum,8);//设定TPDO1子索引数目为2
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
	
	memcpy(uctTxBuf,TPDODON,8);//激活TPDO idD
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
	

	xtTxHeader.StdId = 0x00;
	xtTxHeader.DLC = 2;
	uctTxBuf[0]=0x01;
	uctTxBuf[1]=0x0D;	//启动节点
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);	
}
void TPDOSTART(void)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x60a;//
	xtTxHeader.DLC = 8;
	
	memcpy(uctTxBuf,TPDOstart,8);//激活TPDO idD
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
}





void TPDOSTOP(void)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x60a;
	xtTxHeader.DLC = 8;
	
	memcpy(uctTxBuf,TPDOstop,8);//关闭TPDO idA
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
}

void ALLSTOP(void)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x60C;
	xtTxHeader.DLC = 8;
	
	memcpy(uctTxBuf,JointCmd_MtStop,8);//关闭TPDO idA
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
	

	xtTxHeader.StdId = 0x60A;	
	memcpy(uctTxBuf,JointCmd_MtStop,8);//关闭TPDO idA
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
	
		xtTxHeader.StdId = 0x60B;	
	memcpy(uctTxBuf,JointCmd_MtStop,8);//关闭TPDO idA
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
	
	
		xtTxHeader.StdId = 0x60D;	
	memcpy(uctTxBuf,JointCmd_MtStop,8);//关闭TPDO idA
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
	
	
}


void sync(void)
	{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,0,0,0,0,0,0,0};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;	
	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x80;
	xtTxHeader.DLC = 0;
	
	memcpy(uctTxBuf,TPDOstop,8);//激活TPDO idA
    HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBuf, &ultTxMailBox);	
    osDelay(2);
}







// inputz 是测量值; x 是估计值
double kalmanFilter(double inputz) {
    P += Q;
    // 更新步骤
    double y = inputz - x;
    double S = P + R;  // 测量预测误差的协方差
    double K = P / S;  // 卡尔曼增益
    x = x + K * y;  // 更新状态估计
    P = (1 - K) * P;  // 更新协方差估计
    return x;
}






















