#include "PID.h"

void PID::set_pid(float p,float i,float d,int st,float dz)
{
  kp=p;
  ki=i;
  kd=d;
  sampletime=st;
  dead_zone=dz;
  k[0]=kp+ki*sampletime+kd/sampletime;
  k[1]=kp+2*kd/sampletime;
  k[2]=kd/sampletime;
  
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
  err[0]=target-current;
  if(abs(err[0])<dead_zone) //死区
    output=0;
  else
    output+=k[0]*err[0]-k[1]*err[1]+k[2]*err[2];
  
}



