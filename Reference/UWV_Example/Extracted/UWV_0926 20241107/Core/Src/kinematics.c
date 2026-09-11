#include "kinematics.h"
//#include "math.h"

#include  "arm_math.h"
#define pi 3.141591654f
//#define d5 405.0f //挤出机长管610 短管550 剩余工具默认405 无力传感器是358.217.磨刷是520
//const float d5 = 405.0f; //d5根据作业工具变化

const float d5 = 520.0f; //d5根据作业工具变化

#define a2 413.102f
#define a3 62.5f
#define offset2 1.686
#define offset3 -0.900328
#define offset4	0.785398


//字符型 char = 1个字节，范围为：-2^8~2^8-1
//整型 int = 4个字节，范围为：-2^(32-1)~2^(32-1)-1
//单精度浮点数 float = 4个字节，范围为：-2^128~ +2^128-1
//双精度浮点数 double = 8个字节，范围为：-2^1024~ +2^1024-1
//matlab里面模型 关节旋转方向 和实际机械臂是相反的！！！！！！！！
	

void nimatic(float x, float y, float z, endposdef *ret)
{
	float t, k, a, b, c, u;
	float theta1, theta2, theta3;
	float m1 = d5 * sqrt(2) * 0.5;
	//solve theta3		选取转动角度小的为theta3的解
	t = (x * x + y * y + z * z) - (2 * m1 * m1 + a2 * a2 + a3 * a3 + 2 * m1 * a3);
	a = t + 2 * m1 * a2 + 2 * a2 * a3;
	b = -4 * m1 * a2;
	c = t - 2 * m1 * a2 - 2 * a2 * a3;
	float tmp = b * b - 4 * a * c;
	if (tmp < 0)  return;
	u = (-b - sqrt(tmp)) / 2 / a;
	theta3 = 2 * atan2(u, 1);
	//solve theta2   
	t = (m1 + a3) * arm_cos_f32(theta3) + m1 * arm_sin_f32(theta3) + a2;
	k = (m1 + a3) * arm_sin_f32(theta3) - m1 * arm_cos_f32(theta3);
	a = k + z;
	b = -2 * t;
	c = z - k;
	tmp = b * b - 4 * a * c;
	if (tmp < 0)  return;
	u = (-b - sqrt(tmp)) / 2 / a;
	theta2 = 2 * atan2(u, 1) - offset2;
	theta3 -= offset3;
	//solve theta1
	theta1 = atan2(y, x);
	
	//ceshi

	ret->yjd1 = theta1;
	ret->yjd2 = theta2;
	ret->yjd3 = theta3;
	//坐标控制时 末端关节不旋转
	ret->jd1 = -83443.02679286822 * theta1 + spe1;  //90du，改完需要修改正运动学的
  ret->jd2 = -83443.02679286822 * theta2 + spe2; 
  ret->jd3 = -83443.02679286822 * theta3 + spe3; 
}

void tr2eul(endposdef *ret, eulAngle *eul)//转欧拉角
{
	
	float rt1 = ret->yjd1;
	float rt2 = ret->yjd2 + offset2;
	float rt3 = ret->yjd3 + offset3;
	float R13 = cos(rt1) * sin(rt2 + rt3 + offset4);
	float R23 = sin(rt1) * sin(rt2 + rt3 + offset4);
	eul->alpha = atan2(R23, R13);
  float R31 = sin(rt2 + rt3 + offset4);
	float R33 = -cos(rt2 + rt3 + offset4);
	eul->beta = atan2(R31, R33);
	eul->gamma = 0;
	eul->degree_x = -eul->alpha * 180 / pi;
	eul->degree_y = (pi / 2 - eul->beta) * 180 / pi;
	eul->alpha *= 180 / pi;
	eul->beta *= 180 / pi;
	//ret->yjd1 *= 180 / pi;
	//ret->yjd2 *= 180 / pi;
	//ret->yjd3 *= 180 / pi;
}

/*
void  nimatic(float x,float y,float z,endposdef *ret)
{
float pi=3.141591654f; 
float d1=223.5f; 
float  a2=413.1f;//
//float  a3=222.8;	//机械臂原长
//float  a3=370.1;	//	装夹爪后长度
//float  a3=410.1f;	//装笔后长度	
float  a3=295.1f;	//侧笔后长度	300.1
	
float a1temp,a2temp,a3temp,a4temp;	
float c3,s3,j3,c2,s2,j2,c22,s22,s21,j21,j22,j23,j24,j1;
float ftemp[1]={0.1,};
		
//__sqrtf	 216us
//sqrt	     238us	
//arm_sqrt_f32	 215us

a2temp=a2*a2+a3*a3+2*a2*a3 ;	
//a2temp=390787.5169;	//机械臂原长

//arm_sin_f32();
//arm_cos_f32();
//arm_sqrt_f32	
a1temp=x*x+y*y;	
//a2temp=677658.24f;	//装笔后长度


        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		c3=(a1temp+z*z-a2*a2-a3*a3)/(2*a2*a3); // cos(theta3)的值
		s3=-__sqrtf(1-c3*c3); //% % sin(theta3)的值，这里限定了负值，这样解出来，只有一组值，matlab里关节顺时针旋转为负值		
		j3=atan2(s3,c3);  //% 关节3的最终角度，
	    a3temp=a2+a3*c3;	
	    a4temp=z*a3*s3;	
        a1temp=__sqrtf(a1temp);

		c2 =(a1temp*(a3temp)- a4temp ) / (a2temp );//%关节2角度限定-90度到90度，所以值就是正的
		c22 =(-a1temp*(a3temp)- a4temp      ) / (a2temp );//%关节2角度限定-90度到90度，所以值就是正的
		s21=(-a1temp*(a3*s3)   + z*(a3temp) ) / (a2temp );
		s22=(a1temp*(a3*s3)   + z*(a3temp) ) / (a2temp );
		j21=atan2(s22,c2); // %s3为正数
		j22=atan2(s21,c22); 		
		c2 =( a1temp*(a3temp)+ a4temp      ) / (a2temp );//%关节2角度限定-90度到90度，所以值就是正的
		c22 =(-a1temp*(a3temp)+ a4temp      ) / (a2temp );//%关节2角度限定-90度到90度，所以值就是正的
		s21=(a1temp*(a3*s3)   + z*(a3temp) ) / (a2temp );
		s22=(-a1temp*(a3*s3)   + z*(a3temp) ) / (a2temp );
		j23=atan2(s22,c2);  //%s3为负数
		j24=atan2(s21,c22);  
////-------------------------------------------------------------------	

//		c3=(a1temp+z*z-a2*a2-a3*a3)/(2*a2*a3); // cos(theta3)的值
//		s3=-sqrt(1-c3*c3); //% % sin(theta3)的值，这里限定了负值，这样解出来，只有一组值，matlab里关节顺时针旋转为负值		
//		j3=atan2(s3,c3);  //% 关节3的最终角度，
//	   
// 	    a3temp=a2+a3*c3;	
//	    a4temp=z*a3*s3;	
//        a1temp=sqrt(a1temp);

//		c2 =(a1temp*(a3temp)- a4temp ) / (a2temp );//%关节2角度限定-90度到90度，所以值就是正的
//		c22 =(-a1temp*(a3temp)- a4temp      ) / (a2temp );//%关节2角度限定-90度到90度，所以值就是正的
//		s21=(-a1temp*(a3*s3)   + z*(a3temp) ) / (a2temp );
//		s22=(a1temp*(a3*s3)   + z*(a3temp) ) / (a2temp );
//		j21=atan2(s22,c2); // %s3为正数
//		j22=atan2(s21,c22); 		
//		c2 =( a1temp*(a3temp)+ a4temp      ) / (a2temp );//%关节2角度限定-90度到90度，所以值就是正的
//		c22 =(-a1temp*(a3temp)+ a4temp      ) / (a2temp );//%关节2角度限定-90度到90度，所以值就是正的
//		s21=( a1temp*(a3*s3)   + z*(a3temp) ) / (a2temp );
//		s22=(-a1temp*(a3*s3)   + z*(a3temp) ) / (a2temp );
//		j23=atan2(s22,c2);  //%s3为负数
//		j24=atan2(s21,c22);  



//		

     if(  j23>((-pi)/2)  && j23<(pi)  ) // % 判断关节2的最终角度是不是在-90度到90度，是就赋值，负载关节
		  { j2=j23; }//% 关节2的最终角度，		
	else if( j24<(pi)&&j24>(-pi/2))
		   {j2=j24;} //% 关节2的最终角度，		  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
     j1=atan2(y,x);// %第一个解，底座关节，单位是弧度
///////////////////////////////////////////////////////////////////////////////////////////////////////////////	
 
//0      250000
//45度  0.7853981635弧度  250000+65536   
//-45度                   250000-65536
//		  
ret->yjd1=j1;
ret->yjd2=j2;
ret->yjd3=j3;
		   
			 
			   ret->jd1=-83443.02679286822*j1+231000;  //朝前，改完需要修改正运动学的
     // ret->jd1=-83443.02679286822*j1+362000; //角度90都朝左
			 
      ret->jd2=-83443.02679286822*j2+410000; 
			    //  ret->jd3=-83443.02679286822*j3+235000; //垂直笔长度
      ret->jd3=-83443.02679286822*j3+238000; 
      ret->jd4=0; 

}
*/





float kaerman(float input)
{


	
}



























