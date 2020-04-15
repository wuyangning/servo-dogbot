#include "Arduino.h"
#ifndef PID_h
#define PID_h

class PID
{
  public:
    int sampletime;//采样周期
    float target; //设定值
    float current;//输入
    float output;//输出
    
    void set_pid(float,float,float,int,float);//设定pid值
    void print_pid();//串口输出pid
    void compute();//计算pid
    
  private:
    float kp;
    float ki;
    float kd;
    float dead_zone;

  
    
    float k[3];//系数
    float err[3];//三次的误差
 
};

#endif


