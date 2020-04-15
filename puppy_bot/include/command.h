#include "Arduino.h"
#include "kine.h"

#define cm_start 's' //起始符
#define cm_end '\n' //终止符
#define cm_sep '\n' //分割符


extern KINE leg[4];
extern const uint8 servo_num[8];

void init_command(); //清空指令

void cli_loop();

void process_command();

double key_value(char key);//提取字符后的浮点数

bool command_seen(char key);//判断是否存在字符