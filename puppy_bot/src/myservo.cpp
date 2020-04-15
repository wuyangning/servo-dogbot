#include "myservo.h"

void init_servo()
{
    Timer2.setPeriod(4000);
    Timer3.setPeriod(4000); //PWM:250Hz

    for(uint8 i=0;i<8;i++)
    {
        pinMode(servo_num[i],PWM);
        //pwmWrite(servo_num[i],21000);
    }
    pinMode(PB12,OUTPUT);
    digitalWrite(PB12,HIGH);
}

void set_joint_rad(uint8 num,float radian)
{
    int pul=int(k[num]*pulse_per_rad*radian+b[num]);
    pwmWrite(servo_num[num],pul);

}