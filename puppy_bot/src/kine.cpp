#include "kine.h"



void KINE::servo_attach(byte pin_thigh,byte pin_shank)
{
	servo0=pin_thigh;
	servo1=pin_shank;

}

void KINE::mov(float vx,float vz)
{
  //增量运动
  cx+=vx;
  cz+=vz;
  IK();
  set_joint_rad(servo0,rad0);
  set_joint_rad(servo1,rad1);


}


void KINE::set_pose(float rx,float rz)
{
  cx=rx;
  cz=rz;

  IK();
  set_joint_rad(servo0,rad0);
  set_joint_rad(servo1,rad1);
}


void KINE::line_move(float rx,float rz)
{
  //1mm一个插补点
  //float seg=1;
  float distance=sqrt(rx*rx+rz*rz);
  float seg_x=rx/distance;
  float seg_z=rz/distance;
  for(int i=0;i<int(distance);i++)
  {
    mov(seg_x,seg_z);
    delay(1);
  }
}


void KINE::FK()
{
  //正运动学
  float X2 = L2*cos(rad1)+X1;
  float Z2 = L2*sin(rad1)+Z1;

  float X4 = L1*cos(rad1);
  float Z4 = L1*sin(rad1);
  
  float ka = 2*(X4-X2)*L4;
  float kb = 2*(Z4-Z2)*L4;
  float kc = pow(X4-X2,2)+pow(Z4-Z2,2)+L4*L4-L3*L3;
  float theta4 = 2*atan2(kb+sqrt(ka*ka+kb*kb-kc*kc),ka-kc);

  cx = L5*cos(theta4)-X4;
  cz = -L5*sin(theta4)+Z4;
} 


void KINE::IK()
{
  float COS_O = (L1*L1+L5*L5-pow(cx,2)-pow(cz,2))/(2*L1*L5);
  float SIN_O = sqrt(1-COS_O*COS_O);

  rad0 = atan2(cz,-cx)+atan2(L5*SIN_O,L1-L5*COS_O);
  float theta4 = atan2(SIN_O,COS_O)+rad0;

  float X3 = -cx+(L4+L5)*cos(theta4);
  float Z3 = cz+(L4+L5)*sin(theta4);

  float COS_F = (L2*L2+L3*L3-pow(X3-X1,2)-pow(Z3-Z1,2))/(2*L2*L3);
  float SIN_F = sqrt(1-COS_F*COS_F);

  rad1 = atan2(Z3-Z1,X3-X1)+atan2(L3*SIN_F,L2-L3*COS_F); 
}


