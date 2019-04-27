#include "Arduino.h"

class PID
{
  public:
    int sampletime;//采样周期
    
    void set_pid(float,float,float,int);//设定pid值
    void pid_attach(float *,float*,float*);//绑定pid变量，输入输出和设定值
    void print_pid();//串口输出pid
    void compute();//计算pid
    
  private:
    float kp;
    float ki;
    float kd;

    float *setpoint; //设定值
    float *input;//输入
    float *output;//输出
    
    float k[3];//系数
    float err[3];//三次的误差
    float sum_u;//

  
  
};



