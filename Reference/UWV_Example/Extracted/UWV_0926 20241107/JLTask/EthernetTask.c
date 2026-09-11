/**
  ******************************************************************************
  * @project : HS719ROV-CC
  * @file    : EthernetTask.c
  * @version : 
  * @brief   : 网络数据处理
  * @author  : JL ZhangSheng
  * @contacts: 
  * @address : 
  ******************************************************************************
  * @VersionRecord
  * 
  ******************************************************************************
  * @Copyright
  *
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "jlCAN.h"
#include "gpio.h"
#include "JLGpio.h"

#include "api.h"
#include <string.h>
/* Private includes ----------------------------------------------------------*/
#include "JointTask.h"
#include  "arm_math.h"

#include "CanMsg.h"

#include "kinematics.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define UDP_BUFSIZE					128
#define SET_BUFSIZE					80
#define TEST_STATE  1//测试模式开关0关，1开

/* Private variables ---------------------------------------------------------*/
unsigned char  Note_IP_Address[4]	= {192,168,1,10};		//udp本机地址
unsigned char  Dest_IP_Address[4]	= {192,168,1,9};		//udp连接远端地址

struct netconn *udpRecvConn;
unsigned short UdpRecvPort = 19999;						//udp接收端口号
unsigned int udpRevcLongth = 0;
char udpRecvBuf[UDP_BUFSIZE] = {0,};					//udp协议接收缓冲

struct netconn *udpSendConn;
unsigned short UdpSendPort = 20000;						//udp发送端口号
unsigned int udpSendLongth = 0;							//udp需要发送数据长度
char udpSendBuf[UDP_BUFSIZE] = {0,};					//udp协议发送缓冲
unsigned int udpSetFbLongth = 0;						//udp设置回令数据长度
char udpSetFbBuf[200] = {0,};					//udp协议设置回令缓冲

struct netconn *udpSetConn;
unsigned short UdpSetPort = 20001;						//udp接收设置端口号
unsigned int udpSetLongth = 0;							//udp需要发送数据长度
char udpSetBuf[SET_BUFSIZE] = {0,};						//udp协议接收缓冲
unsigned char  mxLwipInitFlag = 1;						//udp初始化标识；1:未完成初始化,0:初始化完成

//----------------------------------------------------------
extern float real_odo1;
extern float real_odo2;
extern struct kinematic_model xy;
extern float angX,angY,angZ,angZZ;
//----------------------------------------------------------


/* Private function prototypes -----------------------------------------------*/
void EthRecvDecode(void);
void EthSetDecode(void);//接收UDP发送过来的数据并解码，包括电机设置，电机位置，电流速度，电机电流。
extern void MX_LWIP_Init(void);
/* Private user code ---------------------------------------------------------*/


/**
  * @brief  网络接收任务19999端口收到的数据
  * @param  argument	
  * @retval NULL
这个任务是接收19999端口数据的
**/
void EthRecvTask(unsigned char *argument)
{		
	static struct netbuf  *recvbuf;
	ip4_addr_t destipaddr;
	struct pbuf *q;
	
	MX_LWIP_Init();
	
	udpRecvConn = netconn_new(NETCONN_UDP);  //创建一个UDP链接
	if(udpRecvConn != NULL)  //创建UDP连接成功
	{
		netconn_bind(udpRecvConn,IP_ADDR_ANY,UdpRecvPort);
		IP4_ADDR(&destipaddr,Dest_IP_Address[0],Dest_IP_Address[1], Dest_IP_Address[2],Dest_IP_Address[3]); //构造目的IP地址
		netconn_connect(udpRecvConn,&destipaddr,UdpRecvPort); 	//连接到远端主机
	}
	osDelay(100);
	
	mxLwipInitFlag = 0;
	
	osDelay(100);
	
	for(;;)
	{
		netconn_recv(udpRecvConn,&recvbuf); //接收数据
		if(recvbuf != NULL)          //接收到数据
		{ 
			
			udpRevcLongth=0;  						//udpRevcLongth清零。
			memset(udpRecvBuf,0,UDP_BUFSIZE);  		//数据接收缓冲区清零
			
			for(q=recvbuf->p;q!=NULL;q=q->next)  	//遍历完整个pbuf链表
			{
				if(q->len > (UDP_BUFSIZE-udpRevcLongth))
				{
					memcpy(udpRecvBuf+udpRevcLongth,q->payload,(UDP_BUFSIZE-udpRevcLongth));	//拷贝数据
				}
				else
				{
					memcpy(udpRecvBuf+udpRevcLongth,q->payload,q->len);
				}
				udpRevcLongth += q->len;
				if(udpRevcLongth > UDP_BUFSIZE) 
				{
					break; //超出TCP客户端接收数组,跳出	
				}
			}
			netbuf_delete(recvbuf);      //删除buf
			
			EthRecvDecode();
		}
		osDelay(500);
	}
}



/**
  * @brief  网络设置任务，接收udp发送过来的数据并解析，发送can'，或者pwm
  * @param  argument，20001端口	收到的数据
  * @retval NULL

**/
void EthSetTask(unsigned char *argument)
{
	static struct netbuf  *recvbuf;
	ip4_addr_t destipaddr;
	struct pbuf *q;	

	while(mxLwipInitFlag)		//等待LWIP初始化完成
	{
		osDelay(1);
	}

	udpSetConn = netconn_new(NETCONN_UDP);  //创建一个UDP链接
	if(udpSetConn != NULL)  //创建UDP连接成功
	{
		netconn_bind(udpSetConn,IP_ADDR_ANY,UdpSetPort);
		IP4_ADDR(&destipaddr,Dest_IP_Address[0],Dest_IP_Address[1], Dest_IP_Address[2],Dest_IP_Address[3]); //构造目的IP地址
		netconn_connect(udpSetConn,&destipaddr,UdpSetPort); 	//连接到远端主机
	}

	osDelay(200);

	for(;;)
	{
		
		
		netconn_recv(udpSetConn,&recvbuf); //接收数据
		if(recvbuf != NULL)          //接收到数据
		{ 
			udpSetLongth=0;  						//udpRevcLongth清零。
			memset(udpSetBuf,0,SET_BUFSIZE);  		//数据接收缓冲区清零
			
			for(q=recvbuf->p;q!=NULL;q=q->next)  	//遍历完整个pbuf链表
			{
				if(q->len > (SET_BUFSIZE-udpSetLongth))
				{
					memcpy(udpSetBuf+udpSetLongth,q->payload,(SET_BUFSIZE-udpSetLongth));	//拷贝数据
				}
				else
				{
					memcpy(udpSetBuf+udpSetLongth,q->payload,q->len);
				}
				udpSetLongth += q->len;
				if(udpSetLongth > SET_BUFSIZE)
				{
					break; //超出TCP客户端接收数组,跳出	
				}
			}
			netbuf_delete(recvbuf);      //删除buf
			
			EthSetDecode();
		}
		osDelay(2);
	}	
}



/**
  * @brief  网络发送任务，向20001端口发送数据，就是上位机，20ms一次
  * @param  argument	
  * @retval NULL
**/


const float32_t dd1=223.5f; 
const float32_t  aa2=413.1f;//
const float32_t  aa3=295.1f;	//装侧笔后长度	

void EthSendTask(unsigned char *argument)
{
	float q1, q2, q3, q4;
	float sin1, cos1, sin2, cos2, sin23, cos23, sin234, cos234;
	
	uint32_t xuhao=0;
	
	float c1,c2,c3,c4,dy48;	
	float fx,fy,fz,fxt,fyt,fzt;	
	

	
	static struct netbuf  *sentbuf;
	ip4_addr_t destipaddr;
	
	uint8_t tempBuf[40]={0};
	
	while(mxLwipInitFlag)		//等待LWIP初始化完成
	{
		osDelay(1);
	}

	udpSendConn = netconn_new(NETCONN_UDP);  //创建一个UDP链接
	if(udpSendConn != NULL)  //创建UDP连接成功
	{
		netconn_bind(udpSendConn,IP_ADDR_ANY,UdpSendPort);
		IP4_ADDR(&destipaddr,Dest_IP_Address[0],Dest_IP_Address[1], Dest_IP_Address[2],Dest_IP_Address[3]); //构造目的IP地址
		netconn_connect(udpSendConn,&destipaddr,UdpSendPort); 	//连接到远端主机
	}

	osDelay(100);

  for(;;)

	{
		//机械臂底座NowP[13]
		//ret->jd1 = -83443.02679286822 * theta1 + 231000;
    //ret->jd2 = -83443.02679286822 * theta2 + 272430; 
    //ret->jd3 = -83443.02679286822 * theta3 + 365722; 
    //ret->jd4 = 0; 
		//q1(底座)~q4：关节角度
		//q1~q3变，q4不变取0；
		q1 = (spe1 - NowP[13]) / 83443.02679286822f;
		q2 = (spe2 - NowP[12]) / 83443.02679286822f;
		q3 = (spe3 - NowP[11]) / 83443.02679286822f;
		q4 = (spe4-NowP[10]) / 83443.02679286822f;
	
		c1=NowC[10];    //A关节
		c1=(c1/10000)*33;	
		
		c2=NowC[11];//B关节
		c2=(c2/10000)*33;	
		
		c3=NowC[12];
		c3=(c3/10000)*69;		

		c4=NowC[13];		
		c4=(c4/10000)*69;
		
		dy48=NowV[0];//借用下速度变量当电压用
		dy48=dy48/1000;//单位V；
		
		
		//	fxt=	NowT[0];
		//	fxt=fxt/1000;	
		//	fyt=	NowT[1];
		//	fyt=fyt/1000;		
		//	fzt=	NowT[2];
		//	fzt=fzt/1000;
	
		fx=	NowT[3];
		fx=fx/1000;	
		if( (fx>200)|| (fx<-200)){fx=3000;}
	
		fy=	NowT[4];
		fy=fy/1000;		
		if( (fy>200)|| (fy<-200)){fy=500;}
		
		fz=	NowT[5];
		fz=fz/1000;	
		if( (fz>400)|| (fz<-400)){fz=700;}

		//float sin1, cos1, cos2, sin23, cos23, sin234, cos234;
		//#define offset2 1.686
		//#define offset3 -0.900328
		//#define offset4	0.785398
		sin1 = arm_sin_f32(q1);
		cos1 = arm_cos_f32(q1);
		sin2 = arm_sin_f32(q2 + 1.686);   					// offset2 = 1.686
		cos2 = arm_cos_f32(q2 + 1.686);
		sin23 = arm_sin_f32(q2 + q3 + 0.785672);		// offset2 + offset3 = 1.686 - 0.900328 = 
		cos23 = arm_cos_f32(q2 + q3 + 0.785672);
		sin234 = arm_sin_f32(q2 + q3 + 1.57107); 		// offset2 + offset3 + offset4 = 1.686 - 0.900328 + 0.785398 = 
		cos234 = arm_cos_f32(q2 + q3 + 1.57107);

		//x = d_5 cosθ_1 sin(θ_2+θ_3+θ_4) + a_2 cosθ_1 cosθ_2 + a_3 cosθ_1 cos(θ_2+θ_3)
		//y = d_5 sinθ_1 sin(θ_2+θ_3+θ_4) + a_2 sinθ_1 cosθ_2 + a_3 sinθ_1 cos(θ_2+θ_3)
		//z = d_1 + a_2 sinθ_2 - d_5 cos(θ_2+θ_3+θ_4) + a_3 sin(θ_2+θ_3)
		eX = d5 * cos1 * sin234 + a2 * cos1 * cos2 + a3 * cos1 * cos23;
		eY = d5 * sin1 * sin234 + a2 * sin1 * cos2 + a3 * sin1 * cos23;
		eZ = a2 * sin2 - d5 * cos234 + a3 * sin23;

					
		sprintf(tempBuf, "A:%07.2f,%07.2f,%07.2f,%07.2f,", q1*57.296, q2*57.296, q3*57.296,q4*57.296);   //34
		memcpy(udpSetFbBuf, tempBuf, 34);
	
		sprintf(tempBuf, "P:%07.2f,%07.2f,%07.2f,", eX, eY, eZ);   //26
		memcpy(udpSetFbBuf + 34, tempBuf, 26);
	
		sprintf(tempBuf, "F:%06.1f,%06.1f,%06.1f,%06.1f,", -fx, fy, fz,angZZ);   //23+7
		memcpy(udpSetFbBuf + 34 + 26, tempBuf, 30);
			
//		sprintf(tempBuf, "C:%07.3f,%07.3f,%07.3f,%07.3f,", c1, c2, c3, c4);  //34
//		memcpy(udpSetFbBuf + 34 + 26 + 23, tempBuf, 34);
//			
//		sprintf(tempBuf, "LP:%07.2f,%07.2f,%07.2f,", xy.position[0], xy.position[1], xy.position[2]);  //27
//		memcpy(udpSetFbBuf + 34 + 26 + 23 + 34, tempBuf, 27);
			
	//	sprintf(tempBuf, "LC:%07.3f,%07.3f,V:%04.1f,", curDataBuf[0], curDataBuf[1], dy48);  //26
//		memcpy(udpSetFbBuf + 34 + 26 + 23 + 34 + 27, tempBuf, 26);
			
		udpSetFbLongth = 34 + 26 + 30 ;//+ 34 + 27 ;//+ 26;   //170
			
		sentbuf = netbuf_new();
		netbuf_alloc(sentbuf,udpSetFbLongth);
		memcpy(sentbuf->p->payload,(void*)udpSetFbBuf,udpSetFbLongth);
		netconn_send(udpSetConn,sentbuf);  	//将netbuf中的数据发送出去
		netbuf_delete(sentbuf);      		//删除buf
			
		memset(udpSetFbBuf,0,udpSetFbLongth);  //数据发送缓冲区清零
		udpSetFbLongth = 0;

    NowC[2]=NowC[2]+1;
		if(NowC[2]>=100){NowC[2]=10;  NowC[0]=1000;NowC[1]=1000;}//1000ms，断网保护，

    osDelay(10);
	}	
}


unsigned int StringToUint(char *_buf)//以逗号作为分隔符
{
	unsigned char _index = 0;
	unsigned int  _temp = 0;
	while(_buf[_index] !=',')
	{
		if((_buf[_index] <= '9') && (_buf[_index] >= '0'))
		{
			_temp = _temp*10 + _buf[_index] - '0';
		}
		_index++;
	}
	return _temp;
}
//接收UDP发送过来的数据，并按照协议解释
int8_t testflag=0;

uint16_t tools=0;

void EthSetDecode(void)
{	
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBufE[8] = {0,0,0,0,0,0,0,0};
//	char xyzBuf[6] = {8,8,8,8,8,8};
	
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
//	unsigned int  _temp = 0;
	int32_t  _tempXYZ=0;
	int16_t  _tempPWM = 0;	
//	int16_t  chongxiPWM = 0;
//	int16_t  moshuaPWM = 0;	
//	int16_t  qiegePWM = 0;	
//	int16_t  jichuPWM = 0;		
//	int16_t  zuantouPWM = 0;	

	xtTxHeader.IDE = CAN_ID_STD;
	xtTxHeader.RTR = CAN_RTR_DATA;
	xtTxHeader.StdId = 0x600;
	xtTxHeader.DLC = 8;
	
//0	//4/////9///13////////22////////31//    39/40,SXYZ解析udp发送过来的机械臂目标末端位置48，LDC是49开始， 70个字节长度//////////////////%+f
//SXYZ:±123456,±123456,±123456,SQ_1000,‘J’ ‘Z’ ‘H’ 0X01 0x06 0xa4 0x02 0x09,LDC_1500,1500,DENG:0/
	if((udpSetBuf[0] == 'S') && (udpSetBuf[1] == 'X') && (udpSetBuf[28] == ','))
	{
		//111111——————————————————————————————------------------------
		ContMode=1;		
		if(ContMode!=lastMode){ lastMode=ContMode; baseX=eX;baseY=eY;baseZ=eZ;  }//切换控制模式后，获取切换时刻的实时坐标，否则会出现较大位置偏移，手控器切换时的位置和机械臂的位置不能对应，。	
        if(udpSetBuf[5] == '-') 
     {_tempXYZ = ~StringToUint(udpSetBuf+6);	}
		else                   
     {_tempXYZ = StringToUint(udpSetBuf+6);	}		 	
		goalx=_tempXYZ;//手控器原始数据0.217483，单位是米。发过来的字符串是+0217483，除以1000以后，单位是毫米
	    goalx=goalx/1000;				
		goalx=baseX+goalx;	
		if(udpSetBuf[13] == '-')
		{
			_tempXYZ = ~StringToUint(udpSetBuf+14);
		}
		else
		{
			_tempXYZ = StringToUint(udpSetBuf+14);
		}
		goaly=_tempXYZ;
	    goaly=goaly/1000;
		goaly=baseY+goaly;	
		
		if(udpSetBuf[21] == '-')
		{
			_tempXYZ = ~StringToUint(udpSetBuf+22);
		}
		else
		{
			_tempXYZ = StringToUint(udpSetBuf+22);
		}
		goalz=_tempXYZ;
		goalz=goalz/1000;
		goalz=baseZ+goalz;
	}
else if((udpSetBuf[0] == 'P') && (udpSetBuf[1] == 'X') && (udpSetBuf[28] == ','))
 {
 		//111111——————————————————————————————------------------------
		ContMode=2;		
		if(ContMode!=lastMode){ lastMode=ContMode; baseX=eX;baseY=eY;baseZ=eZ;  }//切换控制模式后，切换时刻的实时坐标只赋值一次。	
       
   		if(udpSetBuf[5] == '-')
         {_tempXYZ = ~StringToUint(udpSetBuf+6);	}
	    	else                   
      {_tempXYZ = StringToUint(udpSetBuf+6);	}		 	
	   	goalx=_tempXYZ;//发过来的字符串是+123456，除以100以后，1234.56单位是毫米
	    goalx=goalx/1000;				

		if(udpSetBuf[13] == '-')
		{
			_tempXYZ = ~StringToUint(udpSetBuf+14);
		}
		else
		{
			_tempXYZ = StringToUint(udpSetBuf+14);
		}
		  goaly=_tempXYZ;
	    goaly=goaly/1000;

		
		if(udpSetBuf[21] == '-')
		{
			_tempXYZ = ~StringToUint(udpSetBuf+22);
		}
		else
		{
			_tempXYZ = StringToUint(udpSetBuf+22);
		}
		goalz=_tempXYZ;
		goalz=goalz/1000;

 }	  //180.00  度 /字节长度32。
//A-12345,-18000, 01800, 01800，SQ_1000,1000,‘J’ ‘Z’ ‘H’ 0X01 0x06 0xa4 0x02 0x09,LDC_1500,1500,DENG:0/
else if((udpSetBuf[0] == 'A') && (udpSetBuf[7] == ',') && (udpSetBuf[14] == ','))
 {
 		//111111——————————————————————————————------------------------
		ContMode=3;		

		if(udpSetBuf[1] == '-')
		    {_tempXYZ = ~StringToUint(udpSetBuf+2);		}
		else{_tempXYZ = StringToUint(udpSetBuf+2);		}	 	 
        angA=_tempXYZ;
		angA=angA/5729.5779513;//角度转换为弧度，/57.295779513后.  转换为编码器绝对位置值，即19位编码器的脉冲值0-576200，本次转换在JointTask里面执行
		
		
			if(udpSetBuf[8] == '-')
		    {_tempXYZ = ~StringToUint(udpSetBuf+9);		}
		    else{_tempXYZ = StringToUint(udpSetBuf+9);		}		
        angB=_tempXYZ;
		angB=angB/5729.5779513;
	 
			if(udpSetBuf[15] == '-')
		    {_tempXYZ = ~StringToUint(udpSetBuf+16);		}
		    else{_tempXYZ = StringToUint(udpSetBuf+16);		}			
        angC=_tempXYZ;
		angC=angC/5729.5779513;
			
			if(udpSetBuf[22] == '-')
		    {_tempXYZ = ~StringToUint(udpSetBuf+23);		}
		    else{_tempXYZ = StringToUint(udpSetBuf+23);		}		 
		
        angD=_tempXYZ;
		angD=angD/5729.5779513;//这是最后末端关节 
 }

else if((udpSetBuf[0] == 'S') && (udpSetBuf[1] == 'T') && (udpSetBuf[3] == 'P'))//急停指令
      {
			
		ContMode=4;	
			
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);//关灯					
				
		NowC[0]=1000;//遥控器上下通道值.电机停车	
		NowC[1]=1000;//遥控器左右通道值，电机停转		
				
			} 
			
	else if((udpSetBuf[0] == 'F') && (udpSetBuf[1] == 'X') && (udpSetBuf[3] == 'P'))//X方向力控置,FXOP:10,0100,0100,0010,后面是pid参数。
      {
			
		ContMode=5;	
			
		   		if(udpSetBuf[5] == '-')
         {_tempXYZ = ~StringToUint(udpSetBuf+6);	}//到逗号为止
	    	else                   
      {_tempXYZ = StringToUint(udpSetBuf+5);	}		//之前的协议有+号，所以是从6开始的
	   	NowT[15]=_tempXYZ;//发过来的字符串是+123456，除以100以后，1234.56单位是毫米
			

		   		if(udpSetBuf[8] == '-')
         {_tempXYZ = ~StringToUint(udpSetBuf+9);	}//到逗号为止
	    	else                   
      {_tempXYZ = StringToUint(udpSetBuf+8);	}		 	
	  	Fkp=_tempXYZ;		
	   	Fkp=Fkp/100;   //发过来的字符串是0100，除以100以后，就是1.0
			
		   		if(udpSetBuf[13] == '-')
         {_tempXYZ = ~StringToUint(udpSetBuf+14);	}//到逗号为止
	    	else                   
       {_tempXYZ = StringToUint(udpSetBuf+13);	}		
	  	Fki=_tempXYZ;			
	  	Fki=Fki/100;   //发过来的字符串是0100，除以100以后，就是1.0	
				
				if(udpSetBuf[18] == '-')
         {_tempXYZ = ~StringToUint(udpSetBuf+19);	}//到逗号为止
	    	else                   
      {_tempXYZ = StringToUint(udpSetBuf+18);	}				
      Fkd=_tempXYZ;				
	  	Fkd=Fkd/100;   //发过来的字符串是0100，除以100以后，就是1.0		
			
			} 
			
	else if((udpSetBuf[0] == 'F') && (udpSetBuf[1] == 'Z') && (udpSetBuf[3] == 'P'))//Z方向力控制，FXOP:10,0050,0050,0005,后面是pid参数。
      {
			
		ContMode=6;	
		   		if(udpSetBuf[5] == '-')
         {_tempXYZ = ~StringToUint(udpSetBuf+6);	}//到逗号为止
	    	else                   
      {_tempXYZ = StringToUint(udpSetBuf+5);	}		//之前的协议有+号，所以是从6开始的
	   	NowT[15]=_tempXYZ;//发过来的字符串是+123456，除以100以后，1234.56单位是毫米
			

		   		if(udpSetBuf[8] == '-')
         {_tempXYZ = ~StringToUint(udpSetBuf+9);	}//到逗号为止
	    	else                   
      {_tempXYZ = StringToUint(udpSetBuf+8);	}		 	
	  	Fkp=_tempXYZ;		
	   	Fkp=Fkp/100;   //发过来的字符串是0100，除以100以后，就是1.0
			
		   		if(udpSetBuf[13] == '-')
         {_tempXYZ = ~StringToUint(udpSetBuf+14);	}//到逗号为止
	    	else                   
       {_tempXYZ = StringToUint(udpSetBuf+13);	}		
	  	Fki=_tempXYZ;			
	  	Fki=Fki/100;   //发过来的字符串是0100，除以100以后，就是1.0	
				
				if(udpSetBuf[18] == '-')
         {_tempXYZ = ~StringToUint(udpSetBuf+19);	}//到逗号为止
	    	else                   
      {_tempXYZ = StringToUint(udpSetBuf+18);	}				
      Fkd=_tempXYZ;				
	  	Fkd=Fkd/100;   //发过来的字符串是0100，除以100以后，就是1.0
					
			} 		
			
	else if((udpSetBuf[0] == 'F') && (udpSetBuf[1] == 'Y') && (udpSetBuf[3] == 'P'))//Z方向力控制，FXOP:10,0050,0050,0005,后面是pid参数。
      {
	 ContMode=7;	
       } 
			
	//222222作业工具——————————————————————————————A-12345,-18000, 01800, 01800，SQ_1000,1000,‘J’ ‘Z’ ‘H’ 0X01 0x06 0xa4 0x02 0x09,LDC_1500,1500,DENG:0/------------------------	
	tools=udpSetBuf[29];
	tools= (tools<<8)  +udpSetBuf[30];	
	switch(tools)
	{
		case 0x5351://从sxyz数组35标号开始 SQ=0x53 51 水枪
			{
			_tempPWM = StringToUint(udpSetBuf+32);
           safeFlag=1;		
           if( (_tempPWM >3000) || (_tempPWM <0 ) ) { _tempPWM=1000;}	
	       TIM4->CCR1=_tempPWM; 	
	
				break;
			}
		case 0x4D53://MS=0x4d 0x53 磨刷
			{

			safeFlag=1;		
			_tempPWM = StringToUint(udpSetBuf+32);	
                    if( (_tempPWM >3000) || (_tempPWM <0 ) )  { _tempPWM=10;}	
	    TIM4->CCR3=_tempPWM; 	
				break;
			}
		case 0x5a54://zt= 0x5a,0x54 钻头 和 磨刷 (从左往右 第五个插头 四芯)ZT_1000,TIM4->CCR3=1500停止;
			{
	    safeFlag=1;		
						_tempPWM = StringToUint(udpSetBuf+32);	
                    if( (_tempPWM >3000) || (_tempPWM <0 ) )  { _tempPWM=10;}	
	    TIM4->CCR3=_tempPWM; 	
				break;
			}
		case 0x5147://'QG'=0x51 0x47 切割 （从左往右 第六个）
			{
	  safeFlag=1;
		_tempPWM = StringToUint(udpSetBuf+32);					
                if( (_tempPWM >3000) || (_tempPWM <0 ) ) { _tempPWM=1000;}	
	  TIM4->CCR4=_tempPWM; 	
				break;
			}
		case 0x4a43://JC=0x4a 0x43 挤出 （第二个 三芯）
			{
				//d5 = 610.0;
	        safeFlag=1;	
		     _tempPWM = StringToUint(udpSetBuf+32);	
            if( (_tempPWM >3000) || (_tempPWM <0 ) ) { _tempPWM=1500;}	 TIM3->CCR3=_tempPWM; 		 
	          if(TIM3->CCR3==1500){  huituiFlag=1; }//挤出回退
				break;
			}
		default:{	break;	}//default也需要加，结束这个switch 

	}
	//333333333夹爪控制——————————————————————————————------------------------		
  if((udpSetBuf[42] == 'J') && (udpSetBuf[43] == 'Z')&& (udpSetBuf[44] == 'H') )//控制夹爪的指令
	{
		uctTxBufE[0] = udpSetBuf[46];
		uctTxBufE[1] = udpSetBuf[47];
		uctTxBufE[2] = udpSetBuf[48];
		uctTxBufE[3] = udpSetBuf[49];		
		xtTxHeader.StdId = 0X01;//夹爪只接收id为0x01的can数据，
		HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBufE, &ultTxMailBox);	
		
	}	

	
   //4444444444履带车————————LDC_1500,1500,DENG:0/———————------------------------	
  if((udpSetBuf[51] == 'L') && (udpSetBuf[52] == '_')&& (udpSetBuf[62] == ',') )//履带车的控制指令，L是第49个
	{
		NowC[2]=0;//收到指令了置0，其他时间在网络发送任务里面加，加了10次还没收到，代表UDP中断，一直保持10.
		
		_tempPWM = StringToUint(udpSetBuf+53);		
		NowC[0]=_tempPWM;//遥控器上下通道值
			_tempPWM = StringToUint(udpSetBuf+58);		
		NowC[1]=_tempPWM;//遥控器左右通道值
	}

 
   //55555555555水下灯—————LDC_1500,1500,DENG:0/———L是第49个———————————------------------------	
 if((udpSetBuf[63] == 'D') && (udpSetBuf[64] == ':')&& (udpSetBuf[66] == '/') )//水下灯控制指令，
	{
	
		 if(udpSetBuf[65]=='0')
       {// TIM3->CCR2=0; 
			   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);//关				 
			 }
		 else if(udpSetBuf[65]=='1')
   { //TIM3->CCR2=3000; 
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);	//开	 
	 }
	 if (udpSetBuf[65] == '3')   //力传感器清零
	 {
		 SetForceZero();
		 //udpSetBuf[65] = '2';
	 }
	 if (udpSetBuf[65] == '4') // 回收开
	 {		
	  TIM4->CCR2 = 2000; 
	 }
	 else if (udpSetBuf[65] == '5') // 回收关
	 {
		 TIM4->CCR2 = 1500;
	 }
		 
	}			

}
/**
  * @brief  网络接收数据解码
  * @param  NULL
  * @retval NULL
**/

void EthRecvDecode(void)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBufE[8] = {0,};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;
	
	if(udpRecvBuf[0] == 0)
		xtTxHeader.IDE = CAN_ID_STD;
	else if(udpRecvBuf[0] == 4)
		xtTxHeader.IDE = CAN_ID_EXT;
	else
		return;
	
	if(udpRecvBuf[1] == 0)
		xtTxHeader.RTR = CAN_RTR_DATA;
	else if(udpRecvBuf[1] == 2)
		xtTxHeader.RTR = CAN_RTR_REMOTE;
	else
		return;
	
	xtTxHeader.ExtId = udpRecvBuf[2]<<24 | udpRecvBuf[3]<<16 | udpRecvBuf[4]<<8 | udpRecvBuf[5];
	xtTxHeader.StdId = xtTxHeader.ExtId & 0x7FF;
	
	xtTxHeader.DLC = udpRecvBuf[6];
	
	memcpy(uctTxBufE,(void*)(udpRecvBuf+7),8);

	HAL_CAN_AddTxMessage(&hcan2, &xtTxHeader, uctTxBufE, &ultTxMailBox);	
}
/**
  * @brief  网络发送数据打包
  * @param  NULL
  * @retval 0:打包成功,1:打包失败
**/
void EthSetFbPack(CAN_RxHeaderTypeDef _canRxMsgHeader,unsigned char *_RxBuf);

void EthSendPack(CAN_RxHeaderTypeDef _canRxMsgHeader,unsigned char *_RxBuf)//将can收到的数据返回
{
	EthSetFbPack(_canRxMsgHeader,_RxBuf);//
	
	udpSendBuf[0] = _canRxMsgHeader.IDE;
	udpSendBuf[1] = _canRxMsgHeader.RTR;
	if(_canRxMsgHeader.IDE == CAN_ID_STD)
	{
		udpSendBuf[2] = 0;
		udpSendBuf[3] = 0;
		udpSendBuf[4] = (_canRxMsgHeader.StdId&0x7FF)>>8;
		udpSendBuf[5] = _canRxMsgHeader.StdId&0xFF;
	}
	else
	{
		udpSendBuf[2] = (_canRxMsgHeader.ExtId>>24)&0xFF;
		udpSendBuf[3] = (_canRxMsgHeader.ExtId>>16)&0xFF;
		udpSendBuf[4] = (_canRxMsgHeader.ExtId>>8)&0xFF;
		udpSendBuf[5] = _canRxMsgHeader.ExtId&0xFF;
	}	
	udpSendBuf[6] = _canRxMsgHeader.DLC;	
	memcpy(udpSendBuf+7,_RxBuf,8);	
	udpSendLongth = 15;
}

void UintToString(unsigned int _dat,char *_buf)
{
	char _buf_temp[20] = {0,};
	unsigned char _index = 0,i = 0;
	do
	{
		_buf_temp[_index] = _dat%10 + '0';
		_dat = _dat/10;
		_index++;
	}while(_dat != 0);

	do
	{
		_buf[i] = _buf_temp[_index-1];
		i++;
		_index--;
	}while(_index != 0);
}
unsigned int _temp2 = 0;

//接收can总线收到的数据。并解析打包通过UDP转发给上位机。
void EthSetFbPack(CAN_RxHeaderTypeDef _canRxMsgHeader,unsigned char *_RxBuf)
{   

}
