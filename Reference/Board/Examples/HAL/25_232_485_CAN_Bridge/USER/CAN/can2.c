#include "can2.h"
#include "led.h"

u8 receive_str_can2[128];   //接收缓冲,最大512个字节.
u8 send_str_can2[128];

/*********************************************************************************
*********************启明欣欣 STM32F407应用开发板(高配版)*************************
**********************************************************************************
* 文件名称: can1.c                                                               *
* 文件简述：can配置文件                                                          *
* 创建日期：2017.08.30                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：                                                                     * 
**********************************************************************************
*********************************************************************************/

CAN_HandleTypeDef	CAN2_Handler;     //CAN1句柄
CAN_TxHeaderTypeDef	CAN2_TxHeader;      //发送
CAN_RxHeaderTypeDef	CAN2_RxHeader;      //接收

/****************************************************************************
* 名    称: u8 CAN1_Mode_Init(u8 mode)
* 功    能：CAN初始化
* 入口参数：mode:CAN工作模式;0,普通模式;1,环回模式
* 返回参数：0,成功;
           	其他,失败;
* 说    明：       
****************************************************************************/	
u8 CAN2_Mode_Init(u32 tsjw,u32 tbs2,u32 tbs1,u16 brp,u32 mode)
{
	  CAN_InitTypeDef		CAN2_InitConf;
    
    CAN2_Handler.Instance=CAN2;
	
	  CAN2_Handler.Init = CAN2_InitConf;
	
    CAN2_Handler.Init.Prescaler=brp;				//分频系数(Fdiv)为brp+1
    CAN2_Handler.Init.Mode=mode;					//模式设置 
    CAN2_Handler.Init.SyncJumpWidth=tsjw;			//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1TQ~CAN_SJW_4TQ
    CAN2_Handler.Init.TimeSeg1=tbs1;				//tbs1范围CAN_BS1_1TQ~CAN_BS1_16TQ
    CAN2_Handler.Init.TimeSeg2=tbs2;				//tbs2范围CAN_BS2_1TQ~CAN_BS2_8TQ
    CAN2_Handler.Init.TimeTriggeredMode=DISABLE;	//非时间触发通信模式 
    CAN2_Handler.Init.AutoBusOff=DISABLE;			//软件自动离线管理
    CAN2_Handler.Init.AutoWakeUp=DISABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN2_Handler.Init.AutoRetransmission=ENABLE;	//禁止报文自动传送 
    CAN2_Handler.Init.ReceiveFifoLocked=DISABLE;	//报文不锁定,新的覆盖旧的 
    CAN2_Handler.Init.TransmitFifoPriority=DISABLE;	//优先级由报文标识符决定 
	
    if(HAL_CAN_Init(&CAN2_Handler)!=HAL_OK)			//初始化
		return 1;
    return 0;
}   
 
//CAN底层驱动，引脚配置，时钟配置，中断配置
//此函数会被HAL_CAN_Init()调用
//hcan:CAN句柄
//void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
//{
//    GPIO_InitTypeDef GPIO_Initure;
//    
//    __HAL_RCC_CAN1_CLK_ENABLE();                //使能CAN1时钟
//    __HAL_RCC_GPIOA_CLK_ENABLE();			    //开启GPIOA时钟
//	
//    GPIO_Initure.Pin=GPIO_PIN_11|GPIO_PIN_12;   //PA11,12
//    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //推挽复用
//    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
//    GPIO_Initure.Speed=GPIO_SPEED_FAST;         //快速
//    GPIO_Initure.Alternate=GPIO_AF9_CAN1;       //复用为CAN1
//    HAL_GPIO_Init(GPIOA,&GPIO_Initure);         //初始化
//}

void CAN2_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&CAN2_Handler, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    while(1)
	  {
	  }
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&CAN2_Handler) != HAL_OK)
  {
    /* Start Error */
    while(1)
	  {
	  }
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&CAN2_Handler, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    while(1)
	  {
	  }
  }

  /*##-5- Configure Transmission process #####################################*/
  CAN2_TxHeader.StdId = 0x321;
  CAN2_TxHeader.ExtId = 0x01;
  CAN2_TxHeader.RTR = CAN_RTR_DATA;
  CAN2_TxHeader.IDE = CAN_ID_STD;
  CAN2_TxHeader.DLC = 2;
  CAN2_TxHeader.TransmitGlobalTime = DISABLE;
}

/****************************************************************************
* 名    称: u8 CAN1_Send_Msg(u8* msg,u8 len)
* 功    能：can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
* 入口参数：len:数据长度(最大为8)				     
            msg:数据指针,最大为8个字节.
* 返回参数：0,成功;
           	其他,失败;
* 说    明：       
****************************************************************************/		
u8 CAN2_Send_Msg(u8* msg,u8 len)
{	
    u8 i=0;
	u32 TxMailbox;
	u8 message[8];
    CAN2_TxHeader.StdId=0X12;        //标准标识符
    CAN2_TxHeader.ExtId=0x12;        //扩展标识符(29位)
    CAN2_TxHeader.IDE=CAN_ID_STD;    //使用标准帧
    CAN2_TxHeader.RTR=CAN_RTR_DATA;  //数据帧
    CAN2_TxHeader.DLC=len;                
    for(i=0;i<len;i++)
    {
		message[i]=msg[i];
	}
    if(HAL_CAN_AddTxMessage(&CAN2_Handler, &CAN2_TxHeader, message, &TxMailbox) != HAL_OK)//发送
	{
		return 1;
	}
	while(HAL_CAN_GetTxMailboxesFreeLevel(&CAN2_Handler) != 3) {}
    return 0;	
}

/****************************************************************************
* 名    称: u8 CAN1_Receive_Msg(u8 *buf)
* 功    能：can口接收数据查询
* 入口参数：buf:数据缓存区;	 			     
* 返回参数：0,无数据被收到;
    		    其他,接收的数据长度;
* 说    明：       
****************************************************************************/	
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	u8	RxData[8];

	if(HAL_CAN_GetRxFifoFillLevel(&CAN2_Handler, CAN_RX_FIFO0) != 1)
	{
		return 0xF1;
	}

	if(HAL_CAN_GetRxMessage(&CAN2_Handler, CAN_RX_FIFO0, &CAN2_RxHeader, RxData) != HAL_OK)
	{
		return 0xF2;
	}
    for(i=0;i<CAN2_RxHeader.DLC;i++)
    buf[i]=RxData[i];
	return CAN2_RxHeader.DLC;
}














