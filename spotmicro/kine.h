#include "Arduino.h"
#ifndef kine_h
#define kine_h
#include <MapleFreeRTOS900.h>

//腿部参数
#define l1 110.0
#define l2 122.0
#define sa_ll (l1*l1+l2*l2) //减少计算
#define ll (2*l1*l2)
//#define offset_d 8.0  //忽略小偏置
#define offset_D 60.0 
#define D_D offset_D*offset_D

#define Bxf  20.0 //前腿的初始位置相对于髋关节（x坐标）
#define By 60.0
#define Bz -170.0
#define Bxb -90.0//后退的初始位置

#define X 0
#define Y 1
#define Z 2
#define hip 0
#define thigh 1
#define shank 2

extern void servo_write(byte,float);


class KINE
{
	public:
		
		float pose[3];
		float rad[3];
		
		void FK(); //运动学正解，暂时没有用到
		void IK(); //运动学逆解
		void servo_attach(byte ,byte ,byte ,bool );//绑定舵机编号，并初始化，最后一个参数是腿的类型，true为前腿
    void mov(float,float,float );//点到点的运动（给的参数是坐标增量）
		void line_move(float ,float ,float );//插值运动，默认精度是1mm，需要大量计算量
	private:
    void move_servo(); //移动各个关节
		byte num_servo[3];
	
	
};


#endif



