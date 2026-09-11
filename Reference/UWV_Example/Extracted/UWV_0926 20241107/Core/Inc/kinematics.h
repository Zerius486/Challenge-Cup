#ifndef __kinematics_H
#define __kinematics_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"

#define pi 3.141591654f
#define a2 413.102f
#define a3 62.5f
#define offset2 1.686
#define offset3 -0.900328
#define offset4	0.785398

// start position encoder DH参数编码器初始位置值
#define spe1 251500  
#define spe2 310000 //218200
#define spe3 368000
#define spe4 164400
extern const float d5;

typedef struct{
uint32_t jd1;
uint32_t jd2;
uint32_t jd3;
int32_t jd4;	
float yjd1;	
float yjd2;		
float yjd3;		
	
       }endposdef;

typedef struct {
	float alpha;
	float beta;
	float gamma;
	float degree_x;
	float degree_y;
}eulAngle;

void nimatic(float x,float y,float z,endposdef *ret);
void tr2eul(endposdef *ret, eulAngle *eul);


#ifdef __cplusplus
}
#endif
#endif /*__ can_H */