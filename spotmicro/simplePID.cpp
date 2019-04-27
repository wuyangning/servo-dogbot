//简易pid，使用增量pid递推。


#include "Arduino.h"
#include "simplePID.h"

void PID::set_pid(float p,float i,float d,int st)
{
  kp=p;
  ki=i;
  kd=d;
  sampletime=st;
  k[0]=kp+ki*sampletime+kd/sampletime;
  k[1]=kp+2*kd/sampletime;
  k[2]=kd/sampletime;
  
}

void PID::pid_attach(float *sp,float *in,float *out)
{
  setpoint=sp;
  input=in;
  output=out;
}
void PID::print_pid()
{
  Serial.print("kp:");
  Serial.print(kp);
  Serial.print("ki:");
  Serial.print(kp);
  Serial.print("kd:");
  Serial.println(kd);
}

void PID::compute()
{
  err[2]=err[1];
  err[1]=err[0];
  err[0]=*setpoint-*input;
  if(abs(err[0])<1) //舵机精度不够，加一个死区
    *output=0;
  else
    *output+=k[0]*err[0]-k[1]*err[1]+k[2]*err[2];
  
}



