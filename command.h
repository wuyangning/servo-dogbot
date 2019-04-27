#include "Arduino.h"
#include "kine.h"

#define cm_start 'G' //起始符
#define cm_end '\n' //结束符
#define cm_sep '\n' //分割符

extern KINE leg_lf;
extern KINE leg_lb;
extern KINE leg_rf;
extern KINE leg_rb;

extern byte walk_x;
extern byte walk_y;
extern byte walk_ro;
extern boolean walking;
extern TaskHandle_t Task2_Handle;

extern float target_roll;
extern float target_pitch;
extern void servo_write(byte,float);
extern void sss(byte,int);
extern void move_platform(float,float,float);
extern void walk(void *);



void init_command();//初始化指令

void get_command();//获取指令并处理

void process_command();//处理指令

double key_value(char key);//获取指令中某个字符后的浮点数，默认0

bool command_seen(char key);//判断是否存在某一个字符




