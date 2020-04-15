#include "Arduino.h"
#ifndef myservo_h
#define myservo_h

#define pulse_per_rad 9200 //每弧度对应的脉宽
#define SERVO_ENABLE_PIN PB12 //用一个场效应管开关使能舵机

//实际输出脉宽=k*pulse_per_rad*关节角+b
const int8_t k[8]={-1,-1,1,1,-1,-1,1,1};
const int b[8]={30252,14572,15268,30393,12695,30730,29939,14620};
//引脚对应的舵机编号
const uint8 servo_num[8]={PA0,PA1,PA2,PA3,PA6,PA7,PB0,PB1};

void init_servo();
void set_joint_rad(uint8,float);

#endif