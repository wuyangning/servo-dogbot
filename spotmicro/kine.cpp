 //单腿运动学
 
 
 #include "Arduino.h"
#include "kine.h"



void KINE::servo_attach(byte s0,byte s1,byte s2,bool type)
{
	num_servo[hip]=s0;
	num_servo[thigh]=s1;
	num_servo[shank]=s2;
  if(type==1)
    pose[X]=Bxf;
  else
    pose[X]=Bxb;
  pose[Y]=By;
  pose[Z]=Bz;
  IK();
  move_servo();
}


void KINE::FK()
{
  pose[X]=l1*cos(rad[1])+l2*cos(rad[1]-rad[2]);
  float R=l1*sin(rad[1])+l2*sin(rad[1]-rad[2]);
  float s=sqrt(R*R+D_D);
  float phi=rad[0]-atan2(R,offset_D);
  pose[Y]=s*cos(phi);
  pose[Z]=s*sin(phi);
  
}

void KINE::IK()
{
	float Rz=sqrt(pose[Y]*pose[Y]+pose[Z]*pose[Z]-D_D);
  rad[0]=atan2(pose[Z],pose[Y])+atan2(Rz,offset_D);
  float cs2=(pose[X]*pose[X]+Rz*Rz-sa_ll)/ll;
  float si2=sqrt(1-cs2*cs2);
  rad[2]=atan2(si2,cs2);
  rad[1]=atan2(l1*si2,l1+l2*cs2)+atan2(Rz,pose[X]);
  
}

void KINE::mov(float vx,float vy,float vz)
{
  pose[X]+=vx;
  pose[Y]+=vy;
  pose[Z]+=vz;
  IK();
  move_servo();
}
void KINE::line_move(float vx,float vy,float vz)
{
  //byte seg=1;
  float distance=sqrt(vx*vx+vy*vy+vz*vz);
  float seg_x=vx/distance;
  float seg_y=vy/distance;
  float seg_z=vz/distance;
  for(int i=0;i<int(distance);i++)
  {
    mov(seg_x,seg_y,seg_z);
    //vTaskDelay(2);
   }
}
void KINE::move_servo()
{
  for(byte i=0;i<3;i++)
    servo_write(num_servo[i],rad[i]);
}





