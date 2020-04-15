#include "myimu.h"

static uint8 Re_buf[12];
static uint8 counter=0;


//串口陀螺仪GY953
void init_imu()
{
    Serial3.begin(115200);

    //输出欧拉角
    Serial3.write(0XA5); 
    Serial3.write(0X45);    
    Serial3.write(0XEA);
}

void get_rpy() //获取rpy角
{
  get_data();
  //yall=float(Re_buf[8]<<8|Re_buf[9])/91;
  //if(yall>180)
    //yall-=720;
  ROLL.current=float(Re_buf[6]<<8|Re_buf[7])/98;
  if(ROLL.current>578.7)
    ROLL.current-=668.7;

  PITCH.current=float(Re_buf[4]<<8|Re_buf[5])/98;
  if(PITCH.current>578.7)
    PITCH.current-=668.7;
  PITCH.current=-PITCH.current;

  
 }

 //获取陀螺仪数据
void get_data() //获取一个完整的帧
{
  while (Serial3.available()) 
  {   
    Re_buf[counter]=(unsigned char)Serial3.read();
    if(counter==0&&Re_buf[0]!=0x5A) continue; //如果帧头不对，则重新获取         
    counter++;       
    if(counter==11) //接收到下一帧的帧头为止
    {  
      counter=0; 
      if(Re_buf[0]==0x5A&&Re_buf[11]==0x5A )//如果接收到完整一帧，跳出，否则继续
        break; 
       else
        continue;                       
    }      
  }
}