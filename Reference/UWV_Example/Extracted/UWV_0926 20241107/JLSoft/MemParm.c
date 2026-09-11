#include "MemParm.h"
#include "JLDataType.h"


ExDatTypedef ExDatBuf[ExDatNumb];

char ExDatNameTable[ExDatNumb][11] = {		"01altmdata","02altmtemp","03altmpwen","04dpthdata","05dpthpwen","06exl1pwen","07exl1dimm","08exl2pwen",
											"09exl2dimm","10exl3pwen","11exl3dimm","12exl4pwen","13exl4dimm","14exl5pwen","15exl5dimm","16p330volt",
											"17cellvolt","18pi48cven","19pi48volt","20ccabtemp","21ccableka","22pcabtemp","23pcableka","24eth1pwen",
											"25eth2pwen","26eth3pwen","27eth4pwen","28eth5pwen","29eth6pwen","30flmpsped","31flmpcvlt","32frmpsped",
											"33frmpcvlt","34blmpsped","35blmpcvlt","36brmpsped","37brmpcvlt","38flvpsped","39flvpcvlt","40frvpsped",
											"41frvpcvlt","42blvpsped","43blvpcvlt","44brvpsped","45brvpcvlt","46IanDXAgV","47IanDYAgV","48IanDZAgV",
											"49IanDXAcc","50IanDYAcc","51IanDZAcc","52IanDNSpd","53IanDSSpd","54IanDESpd","55IanDAlti","56IanDLong",
											"57IanDLati","58IanDRoll","59IanDYaw ","60IanDPtch","61IanDHeav","62IanDpwen"};

const OptTypeEm ExDatOpRypeEm[ExDatNumb] = {	DatOp_RO,	DatOp_RO,	DatOp_WO,	DatOp_RO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,
												DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_RO,
												DatOp_RO,	DatOp_WO,	DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_WO,
												DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_RO,	DatOp_WO,	DatOp_RO,
												DatOp_WO,	DatOp_RO,	DatOp_WO,	DatOp_RO,	DatOp_WO,	DatOp_RO,	DatOp_WO,	DatOp_RO,
												DatOp_WO,	DatOp_RO,	DatOp_WO,	DatOp_RO,	DatOp_WO,	DatOp_RO,	DatOp_RO,	DatOp_RO,
												DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_RO,
												DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_RO,	DatOp_WO};
//const OptTypeEm ExDatOpRypeEm[ExDatNumb] = {	DatOp_RO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,
//												DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,
//												DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,
//												DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,
//												DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,
//												DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO,	DatOp_WO};
const DatTypeEm ExDatFbRypeEm[ExDatNumb] = {	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_b,	DatTypeEm_f,	DatTypeEm_b,	DatTypeEm_b,	DatTypeEm_f,	DatTypeEm_b,
												DatTypeEm_f,	DatTypeEm_b,	DatTypeEm_f,	DatTypeEm_b,	DatTypeEm_f,	DatTypeEm_b,	DatTypeEm_f,	DatTypeEm_f,
												DatTypeEm_f,	DatTypeEm_b,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_b,
												DatTypeEm_b,	DatTypeEm_b,	DatTypeEm_b,	DatTypeEm_b,	DatTypeEm_b,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,
												DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,
												DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,
												DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_i,
												DatTypeEm_i,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_f,	DatTypeEm_i,	DatTypeEm_b};



//数据初始化函数
void ExDataInit(void)
{
	unsigned char i = 0;
	for(i = 0; i < ExDatNumb;i++)
	{
		ExDatBuf[i].id = i+1;
		ExDatBuf[i].pName = ExDatNameTable[i];
		ExDatBuf[i].opType = ExDatOpRypeEm[i];
		ExDatBuf[i].dtType = ExDatFbRypeEm[i];
		ExDatBuf[i].bdata = '0';
		ExDatBuf[i].fdata = 0.0f;
		ExDatBuf[i].idata = 0;
	}
	
	ExDatBuf[PIndxIanDPwEn].bdata = '1';
	
//	ExDatBuf[EthPwEn(0)].bdata = '1';
//	ExDatBuf[EthPwEn(1)].bdata = '1';
//	ExDatBuf[EthPwEn(2)].bdata = '1';
//	ExDatBuf[EthPwEn(3)].bdata = '1';
//	ExDatBuf[EthPwEn(4)].bdata = '1';
//	ExDatBuf[EthPwEn(5)].bdata = '1';
//	ExDatBuf[ExLedPwEn(1)].bdata = '1';
//	ExDatBuf[ExLedPwEn(2)].bdata = '1';
//	ExDatBuf[ExLedPwEn(3)].bdata = '1';
//	ExDatBuf[ExLedPwEn(4)].bdata = '1';
//	ExDatBuf[ExLedPwEn(5)].bdata = '1';
	
}

//发送数据打包函数，将一个数据打包成上发的格式，存在tempsendBuf内，以‘;’（分号）结束
unsigned char tempsendBuf[32] = {0,};
unsigned char SendStringCov(ExDatTypedef _dat)
{
	unsigned char i = 0,j = 0;
	unsigned long _temp = 0;
	unsigned char _datBuf[10] = {0,};
	
	for(i = 0;i < 32;i++)
	{
		tempsendBuf[i] = 0;
	}
	i = 0;
	for(i = 0;i < 10;i++)
	{
		tempsendBuf[i] = *(_dat.pName+i);
	}
	
	tempsendBuf[i++] = '=';
	if(_dat.dtType == DatTypeEm_b)
	{
		if(_dat.bdata != '1')
		{
			_dat.bdata = '0';
		}
		tempsendBuf[i++] = _dat.bdata;
	}
	else if(_dat.dtType == DatTypeEm_i)
	{
		if(_dat.idata < 0)
		{
			tempsendBuf[i++] = '-';
			_temp = -_dat.idata;
		}
		else
		{
			_temp = _dat.idata;
		}
		
		if(_temp == 0)
		{
			tempsendBuf[i++] = '0';
		}
		else
		{
			j = 0;
			
			while((_temp!= 0) && (j <= 9))
			{
				_datBuf[j++] = _temp%10 + '0';
				_temp = _temp/10;
			}

			while(j > 0)
			{
				j--;
				tempsendBuf[i++] = _datBuf[j];
			}
		}
	}
	else if(_dat.dtType == DatTypeEm_f)
	{
		if(_dat.fdata < 0)
		{
			tempsendBuf[i++] = '-';
			_temp = (unsigned long)(_dat.fdata*(-1000));
		}
		else
		{
			_temp = (unsigned long)(_dat.fdata*1000);
		}
		if(_temp == 0)
		{
			tempsendBuf[i++] = '0';
		}
		else
		{
			j = 0;
			_datBuf[j++] = _temp%10 + '0';
			_temp = _temp/10;
			_datBuf[j++] = _temp%10 + '0';
			_temp = _temp/10;
			_datBuf[j++] = _temp%10 + '0';
			_temp = _temp/10;
			_datBuf[j++] = '.';
			if(_temp == 0)
			{
				_datBuf[j++] = '0';
			}
			else
			{
				while((_temp != 0) && (j <= 9))
				{
					_datBuf[j++] = _temp%10 + '0';
					_temp = _temp/10;
				}
			}
			while(j > 0)
			{
				j--;
				tempsendBuf[i++] = _datBuf[j];
			}
		}
	}
	tempsendBuf[i++] = ';';
	return i;
}




//extern ExDatTypedef altmdata;				//高度计数据（1）
//extern ExDatTypedef altmtemp;				//高度计温度（2）
//extern ExDatTypedef altmpwen;				//高度计电源使能（3）
//extern ExDatTypedef dpthdata;				//深度计数据（4）
//extern ExDatTypedef dpthpwen;				//深度计电源使能（5）
//extern ExDatTypedef exl1pwen;				//外部灯1电源使能（6）
//extern ExDatTypedef exl1dimm;				//外部灯1调光电压（7）
//extern ExDatTypedef exl2pwen;				//外部灯2电源使能（8）
//extern ExDatTypedef exl2dimm;				//外部灯2调光电压（9）
//extern ExDatTypedef exl3pwen;				//外部灯3电源使能（10）
//extern ExDatTypedef exl3dimm;				//外部灯3调光电压（11）
//extern ExDatTypedef exl4pwen;				//外部灯4电源使能（12）
//extern ExDatTypedef exl4dimm;				//外部灯4调光电压（13）
//extern ExDatTypedef exl5pwen;				//外部灯5电源使能（14）
//extern ExDatTypedef exl5dimm;				//外部灯5调光电压（15）
//extern ExDatTypedef p330volt;				//330V母线电压（16）
//extern ExDatTypedef cellvolt;				//电池电压（17）
//extern ExDatTypedef pi48cven;				//330V转48V使能（18）
//extern ExDatTypedef pi48volt;				//330V转48V电压（19）
//extern ExDatTypedef ccabtemp;				//控制舱温度（20）
//extern ExDatTypedef ccableka;				//控制舱漏水信号（21）
//extern ExDatTypedef pcabtemp;				//电源舱温度（22）
//extern ExDatTypedef pcableka;				//电源舱漏水信号（23）
//extern ExDatTypedef eth1pwen;				//POE1摄像头供电使能（24）
//extern ExDatTypedef eth2pwen;				//POE1摄像头供电使能（25）
//extern ExDatTypedef eth3pwen;				//POE1摄像头供电使能（26）
//extern ExDatTypedef eth4pwen;				//POE1摄像头供电使能（27）
//extern ExDatTypedef eth5pwen;				//POE1摄像头供电使能（28）
//extern ExDatTypedef eth6pwen;				//POE1摄像头供电使能（29）
//extern ExDatTypedef flmpsped;				//前左主推速度值（30）
//extern ExDatTypedef flmpcvlt;				//前左主推控制电压（31）
//extern ExDatTypedef frmpsped;				//前右主推速度值（32）
//extern ExDatTypedef frmpcvlt;				//前右主推控制电压（33）
//extern ExDatTypedef blmpsped;				//后左主推速度值（34）
//extern ExDatTypedef blmpcvlt;				//后左主推控制电压（35）
//extern ExDatTypedef brmpsped;				//后右主推速度值（36）
//extern ExDatTypedef brmpcvlt;				//后右主推控制电压（37）
//extern ExDatTypedef flvpsped;				//前左垂推速度值（38）
//extern ExDatTypedef flvpcvlt;				//前左垂推控制电压（39）
//extern ExDatTypedef frvpsped;				//前右垂推速度值（40）
//extern ExDatTypedef frvpcvlt;				//前右垂推控制电压（41）
//extern ExDatTypedef blvpsped;				//后左垂推速度值（42）
//extern ExDatTypedef blvpcvlt;				//后左垂推控制电压（43）
//extern ExDatTypedef brvpsped;				//后右垂推速度值（44）
//extern ExDatTypedef brvpcvlt;				//后右垂推控制电压（45）

//ExDatTypedef altmdata;				//高度计数据（1）
//ExDatTypedef altmtemp;				//高度计温度（2）
//ExDatTypedef altmpwen;				//高度计电源使能（3）
//ExDatTypedef dpthdata;				//深度计数据（4）
//ExDatTypedef dpthpwen;				//深度计电源使能（5）
//ExDatTypedef exl1pwen;				//外部灯1电源使能（6）
//ExDatTypedef exl1dimm;				//外部灯1调光电压（7）
//ExDatTypedef exl2pwen;				//外部灯2电源使能（8）
//ExDatTypedef exl2dimm;				//外部灯2调光电压（9）
//ExDatTypedef exl3pwen;				//外部灯3电源使能（10）
//ExDatTypedef exl3dimm;				//外部灯3调光电压（11）
//ExDatTypedef exl4pwen;				//外部灯4电源使能（12）
//ExDatTypedef exl4dimm;				//外部灯4调光电压（13）
//ExDatTypedef exl5pwen;				//外部灯5电源使能（14）
//ExDatTypedef exl5dimm;				//外部灯5调光电压（15）
//ExDatTypedef p330volt;				//330V母线电压（16）
//ExDatTypedef cellvolt;				//电池电压（17）
//ExDatTypedef pi48cven;				//330V转48V使能（18）
//ExDatTypedef pi48volt;				//330V转48V电压（19）
//ExDatTypedef ccabtemp;				//控制舱温度（20）
//ExDatTypedef ccableka;				//控制舱漏水信号（21）
//ExDatTypedef pcabtemp;				//电源舱温度（22）
//ExDatTypedef pcableka;				//电源舱漏水信号（23）
//ExDatTypedef eth1pwen;				//POE1摄像头供电使能（24）
//ExDatTypedef eth2pwen;				//POE1摄像头供电使能（25）
//ExDatTypedef eth3pwen;				//POE1摄像头供电使能（26）
//ExDatTypedef eth4pwen;				//POE1摄像头供电使能（27）
//ExDatTypedef eth5pwen;				//POE1摄像头供电使能（28）
//ExDatTypedef eth6pwen;				//POE1摄像头供电使能（29）
//ExDatTypedef flmpsped;				//前左主推速度值（30）
//ExDatTypedef flmpcvlt;				//前左主推控制电压（31）
//ExDatTypedef frmpsped;				//前右主推速度值（32）
//ExDatTypedef frmpcvlt;				//前右主推控制电压（33）
//ExDatTypedef blmpsped;				//后左主推速度值（34）
//ExDatTypedef blmpcvlt;				//后左主推控制电压（35）
//ExDatTypedef brmpsped;				//后右主推速度值（36）
//ExDatTypedef brmpcvlt;				//后右主推控制电压（37）
//ExDatTypedef flvpsped;				//前左垂推速度值（38）
//ExDatTypedef flvpcvlt;				//前左垂推控制电压（39）
//ExDatTypedef frvpsped;				//前右垂推速度值（40）
//ExDatTypedef frvpcvlt;				//前右垂推控制电压（41）
//ExDatTypedef blvpsped;				//后左垂推速度值（42）
//ExDatTypedef blvpcvlt;				//后左垂推控制电压（43）
//ExDatTypedef brvpsped;				//后右垂推速度值（44）
//ExDatTypedef brvpcvlt;				//后右垂推控制电压（45）

