#include "Arduino.h"
#include "kine.h"
#include "PID.h"
#include "myimu.h"
#include "myservo.h"


#define flight_points 20 //腿滞空时的轨迹点数,不可过大，小于127
#define min_flight_period 0.125 //腿部滞空的最短周期占比，用来限制轨迹表的大小
#define trajy_table_size (int)(flight_points/min_flight_period+1) //轨迹表的大小 
#define trajy_table_num 2 //轨迹表的个数

#define ellipse_trajy 1
#define linear_trajy 0

enum gait{TROT,WALK,PACE,PRONK,BOUND,SPACE_WALK};
enum leg{LF,RB,RF,LB};

//轨迹参数
struct trajy_para
{
    float flight_period; //腿腾空的周期占比
    uint8 trajy_type; //滞空的轨迹类型 1为椭圆轨迹，0为两段直线
    float tx; //腿的目标位置
    float tz;
    float hx; //抬腿的最高位置
    float hz; //如果是椭圆轨迹，表示椭圆半短轴
    uint8 num_talbe;//存入的轨迹表

};

void setup_motion();
void trajy_planning(trajy_para p); //单腿轨迹规划
void gait_loop(); //步态循环
void gait_transit(uint8 type,float lift_height); //步态过渡
void rotation_platform(float,float);