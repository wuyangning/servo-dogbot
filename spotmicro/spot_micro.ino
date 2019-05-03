/*
    代码只完成了基本的运动，包括单腿运动学、姿态自稳。也调了一个小跑的步态（同时抬对角线两条腿的），
  理论上可以实现全向运动的，但是舵机响应速度不够，效果不是很理想。弄一个静态的小跑步态应该可以，
  只是走得比较慢。
    程序基于arduino maple,或者说是arduino for stm32。


*/


#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>//pca9685 pwm拓展板库
#include "kine.h"
#include "simplePID.h" 
#include "motion.h"
#include <MapleFreeRTOS900.h>//使用rtos


#define SAMEPLE_TIME 20 //采样周期。ms

//舵机标定值，脉冲宽度=k*关节角+b
//这一步比较麻烦，详细的可以看Adafruit_PWMServoDriver的例程
const int k[16]={-280,-258,0,236,280,299,0,-261,-267,236,0,-251,280,-302,0,226};
const int b[16]={865,1145,0,340,580,20,0,1020,700,200,0,930,605,1260,0,230};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//创建四条腿对象
KINE leg_lf;
KINE leg_lb;
KINE leg_rf;
KINE leg_rb;

//创建两个姿态角对象
PID ROLL;
PID PITCH;

boolean walking=false;//行走的标记

extern void get_command();
extern void walk();
extern void rotation_platform(float,float);

unsigned long current;//当前时间
unsigned long lastime;//上一次时间，PID中使用

float target_roll=0.0;
float target_pitch=0.0;
float roll,add_roll;
float pitch,add_pitch;

//行走的运动量,参数是抬一次腿走的距离。
byte walk_x=30;//
byte walk_y=0;//
byte walk_ro=0;//原地转

TaskHandle_t Task2_Handle; //任务2的句柄

unsigned char Re_buf[11],counter=0;//陀螺仪相关变量

void setup()
{
  pwm.begin();
  pwm.setPWMFreq(120);
  
  Serial.begin(9600);//usb串口
  Serial2.begin(9600);//蓝牙串口
  Serial1.begin(115200);//陀螺仪串口
  
  Serial1.write(0XA5); 
  Serial1.write(0X45);    //初始化GY953
  Serial1.write(0XEA);
 
 //设定必要的pid参数
  ROLL.set_pid(0.08,0.0000,0.02,SAMEPLE_TIME);
  PITCH.set_pid(0.08,0.0000,0.02,SAMEPLE_TIME);
  ROLL.pid_attach(&target_roll,&roll,&add_roll);
  PITCH.pid_attach(&target_pitch,&pitch,&add_pitch);


//绑定各个舵机到，pwm扩展板上的引脚
  leg_lf.servo_attach(8,9,11,1);
  leg_lb.servo_attach(4,5,7,0);
  leg_rf.servo_attach(12,13,15,1);
  leg_rb.servo_attach(0,1,3,0);


  
 //创建两个任务 
  xTaskCreate(balance,
                "Task1",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2, 
                NULL);

    

  xTaskCreate(walk,
                "Task2",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                &Task2_Handle);
                            
	vTaskStartScheduler();

  //vTaskSuspend(Task2_Handle);
  
}


static void walk(void *pvParameters) //行走进程
{  
  for (;;) 
  {   
     vTaskDelay(1);
     //Serial.println("sign");
    if(walking)
    {
        //Serial.println("echo");
        leg_rf.mov(walk_x,-walk_y+walk_ro,20);
        leg_lb.mov(walk_x,walk_y+walk_ro,20);
        vTaskDelay(100);
        leg_lf.mov(-walk_x,-walk_y-walk_ro,0);
        leg_rb.mov(-walk_x,walk_y-walk_ro,0);
        vTaskDelay(100);
        leg_rf.mov(0,0,-20);
        leg_lb.mov(0,0,-20);
        vTaskDelay(100);
      
        leg_lf.mov(walk_x,walk_y+walk_ro,20);
        leg_rb.mov(walk_x,-walk_y+walk_ro,20);
        vTaskDelay(100);
        leg_rf.mov(-walk_x,walk_y-walk_ro,0);
        leg_lb.mov(-walk_x,-walk_y-walk_ro,0);
        vTaskDelay(100);
        leg_lf.mov(0,0,-20);
        leg_rb.mov(0,0,-20);
        vTaskDelay(99);
    } 
              
  }
}

static void balance(void *pvParameters) //平衡进程
{ 
  lastime= xTaskGetTickCount();   
  for (;;) 
  {
    get_command();
    //Serial.println(target_roll);
    current=xTaskGetTickCount();
    if((current-lastime)>=SAMEPLE_TIME)
    {
      get_rpy();
      ROLL.compute();
      PITCH.compute();
      rotation_platform(-add_roll,-add_pitch);
      lastime=current;
    }
  }
}



void loop()//空
{

} 


void servo_write(byte num,float angle)//转动舵机关节角
{
  int pulse=int(k[num]*angle+b[num]);
  pwm.setPWM(num, 0, pulse);

}





//---------------------------陀螺仪-----------------------------
//用的是自带卡尔曼滤波的串口陀螺仪GY953
void get_rpy() //获取rpy角
{
  get_data();
  //yall=float(Re_buf[8]<<8|Re_buf[9])/91;
  //if(yall>180)
    //yall-=720;
  roll=float(Re_buf[6]<<8|Re_buf[7])/91;
  if(roll >180)
    roll-=720;
  roll=-roll;
  pitch=float(Re_buf[4]<<8|Re_buf[5])/91;
  if(pitch>180)
    pitch-=720;
  
 }

 //获取陀螺仪数据
void get_data() //获取一个完整的帧
{
  while (Serial1.available()) 
  {   
    Re_buf[counter]=(unsigned char)Serial1.read();
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





