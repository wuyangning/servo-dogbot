#include "Arduino.h"
#include "kine.h"
extern KINE leg_lf;
extern KINE leg_rf;
extern KINE leg_lb;
extern KINE leg_rb;
extern boolean walking;
void rotation(float,float);//转动平台，暂时不支持yall方向的运动
void move_platform(float,float,float);//移动平台，身体向前后左右上下
void walk(byte,byte,byte);//行走





