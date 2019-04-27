//控制指令


#include "Arduino.h"
#include "command.h"


char command[64];
byte command_length;





void init_command()
{

  for (byte i=0; i<64; i++)//清空指令数组
    command[i] = 0;
  command_length = 0;
}

void get_command()
{
  char c;
  while(Serial2.available() > 0)
  {
    c = Serial2.read();
    if (c != cm_end)
    {
      command[command_length] = c;
      command_length++;
    }
  }
 
  if (command_length && (c == cm_end) )
  {
    process_command();//处理指令
    init_command();//初始化指令
  }
  
}


/*--------------------------------------处理指令---------------------------------------------------*/
//指令以“起始符+数字 字符+数字...+结束符”的形式,起始符后的数字表示指令的类型，之后的字符和数字都是指定一些参数。
//如"M5 X10 Y10 \n"就是改变机器人的姿态到X10度Y10度（rpy角）
void process_command()
{

  byte code = 0;
  
  
  if (command_seen(cm_start) )
  { 
    
    code = (int)key_value(cm_start);
    switch (code)
    {
         
      case 0:
      Serial.println("echo");//测试用
      break;
      
      case 1:
        leg_lf.mov(key_value('X'),key_value('Y'),key_value('Z'));//移动左前腿
      break;
      case 2:
        leg_lb.mov(key_value('X'),key_value('Y'),key_value('Z'));//移动左后腿
      break;
      case 3:
        leg_rf.mov(key_value('X'),key_value('Y'),key_value('Z'));//移动右前腿
      break;
      case 4:
        leg_rb.mov(key_value('X'),key_value('Y'),key_value('Z'));//移动右后腿
      break;
      case 5:
        target_roll=key_value('X'); //设定姿态的目标值
        target_pitch=-key_value('Y');
      break;
      case 6:
        move_platform(key_value('X'),key_value('Y'),key_value('Z'));//移动平台
      break;

      //向前走，先恢复腿的状态，再迈起始步伐，最后开启线程2让机器人走
      case 7:
        //vTaskResume(Task2_Handle);
        leg_lf.servo_attach(8,9,11,1);
        leg_lb.servo_attach(4,5,7,0);
        leg_rf.servo_attach(12,13,15,1);
        leg_rb.servo_attach(0,1,3,0);
        walk_x=key_value('X');
        walk_y=key_value('Y');
        walk_ro=key_value('R');
        leg_lf.mov(0,0,20);
        leg_rb.mov(0,0,20);
        //delay(100);
        leg_lf.mov(walk_x,walk_y+walk_ro,-20);
        leg_rb.mov(walk_x,-walk_y+walk_ro,-20);
        walking=true;
        vTaskResume(Task2_Handle);
        
      break;

      //停止运动，挂起线程
      case 8:
      vTaskSuspend(Task2_Handle);
        walking=false;
        

      break;

      
      

      
    }
   
  }
  
  
}
/*-------------------------------------------处理指令结束----------------------------------------------*/

double key_value(char key)
{
  char temp[10] = "";

  for (byte i=0; i<command_length; i++)
  {
    if (command[i] == key)
    {
      i++;      
      int k = 0;
      while (i < command_length && k < 10)
      {
        if (command[i] == cm_sep || command[i] == cm_end)
          break;

        temp[k] = command[i];
        i++;
        k++;
      }
    return strtod(temp, NULL);
    }
  }
  
  return 0;//默认0
}


bool command_seen(char key)
{
  for (byte i=0; i<command_length; i++)
  {
    if (command[i] == key)
      return true;
  }
  
  return false;
}




