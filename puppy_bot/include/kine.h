#include "Arduino.h"
#include "myservo.h"

#ifndef kine_h
#define kine_h

//机械参数
#define L1 55.0
#define L2 20.0
#define L3 55.0
#define L4 25.0
#define L5 55.0
#define X1 10.5
#define Z1 13.2


#define X 0 
#define Z 1


#define thigh 0 
#define shank 1


extern void set_joint_rad(byte,float);


class KINE
{
	public:
		//当前坐标
		float cx;
		float cz;
		//舵机编号（引脚）
		uint8 servo0;
		uint8 servo1;
		//单腿轨迹的起始点
		float spx;
		float spz;

		uint8 table;
		
		void servo_attach(byte pin_thigh,byte pin_shank);//绑定舵机编号
    	void mov(float vx,float vz);//增量运动
		void line_move(float rx,float rz);//增量直线插补
		void set_pose(float tx,float tz);//移动到指定点
		

	private:
		float rad0;
		float rad1;
		void FK(); //正运动学
		void IK(); //逆运动学

};



#endif



