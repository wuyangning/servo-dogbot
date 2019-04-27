//一些完整的运动

#include "Arduino.h"
#include "motion.h"


void rotation_platform(float rx,float ry)
{

  leg_lf.mov(0,0,rx-ry); 
  leg_rf.mov(0,0,-rx-ry);
  leg_lb.mov(0,0,rx+ry); 
  leg_rb.mov(0,0,-rx+ry);
   
}
void move_platform(float x,float y,float z)//移动平台
{
    leg_lf.mov(-x,-y,-z); 
    leg_rf.mov(-x,y,-z);
    leg_lb.mov(-x,-y,-z); 
    leg_rb.mov(-x,y,-z);
}

void walk(byte vx,byte vy,byte ro)//未完成
{ 
 
}




