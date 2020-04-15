#include "motion.h"


KINE leg[4];
PID ROLL,PITCH;


//LF,RB,RF,LB的步态相位
float phase[4]={0.0,0.0,0.5,0.5}; 
//轨迹表，储存坐标和延迟（速度信息）
float trajy_table[trajy_table_num][trajy_table_size][2]; 
//滞空的平均速度(mm/us)，舵机响应速度越高越接近真实速度
float flight_speed=0.0003; 

//以固定时间间隔执行轨迹表中的点
int point_time;
int period_points;

const unsigned int sample_time=20;
unsigned long current,lastime;


extern void get_rpy();

 
void setup_motion()
{
    init_servo();
    init_imu();

    leg[LF].servo_attach(6,7);
    leg[LB].servo_attach(3,2);
    leg[RF].servo_attach(4,5);
    leg[RB].servo_attach(1,0);

    leg[LF].set_pose(-10,-80);
    leg[LB].set_pose(-30,-80);
    leg[RF].set_pose(-10,-80);
    leg[RB].set_pose(-30,-80);

    for(int i=0;i<4;i++)
    {
        leg[i].spx=leg[i].cx;
        leg[i].spz=leg[i].cz;
        leg[i].table=0;
    }

    ROLL.set_pid(0.02,0.0,0.02,20,0.1);
    PITCH.set_pid(0.03,0.0,0.02,20,0.1);

    ROLL.target=0;
    PITCH.target=0;

} 

void trajy_planning(trajy_para p) 
{
    //滞空为半椭圆轨迹或两段直线
    //直线轨迹是匀速的，椭圆轨迹可以获得一定加减速效果

    period_points=int(flight_points/p.flight_period);
    int prop_points=period_points-flight_points;

    if(p.trajy_type)
    {
        float a=sqrt(p.tx*p.tx+p.tz*p.tz)/2; //椭圆半长轴
        float theta;
        float phi=atan2(p.tz,p.tx);

        for(int i=0;i<flight_points;i++)
        {
            theta=PI-i*PI/flight_points;
            trajy_table[p.num_talbe][i][X]=
                a*cos(theta)*cos(phi)-p.hz*sin(theta)*sin(phi)+p.tx/2;
            trajy_table[p.num_talbe][i][Z]=
                a*cos(theta)*sin(phi)+p.hz*sin(theta)*cos(phi)+p.tz/2;
        }
        point_time=int((PI*p.hz+2*(a-p.hz))/(flight_speed*flight_points));
    }
    else
    {
        float s1=sqrt(p.hx*p.hx+p.hz*p.hz);
        float s2=sqrt(pow(p.tx-p.hx,2)+pow(p.tz-p.hz,2));
        int lift_points=int((s1*flight_points)/(s1+s2));
        int drop_points=flight_points-lift_points;

        float delta_lx=p.hx/lift_points;
        float delta_lz=p.hz/lift_points;
        float delta_dx=(p.tx-p.hx)/drop_points;
        float delta_dz=(p.tz-p.hz)/drop_points;

        for(int i=0;i<lift_points;i++)
        {
            trajy_table[p.num_talbe][i][X]=delta_lx*i;
            trajy_table[p.num_talbe][i][Z]=delta_lz*i;
        }
        for(int i=0;i<drop_points;i++)
        {
            trajy_table[p.num_talbe][i+lift_points][X]=delta_dx*i+p.hx;
            trajy_table[p.num_talbe][i+lift_points][Z]=delta_dz*i+p.hz;
        }

        point_time=int((s1+s2)/(flight_speed*flight_points));
    }
    

    //支撑轨迹为直线

    float delta_px=-p.tx/prop_points;
    float delta_pz=-p.tz/prop_points;

    for(int i=0;i<prop_points;i++)
    {
        trajy_table[p.num_talbe][i+flight_points][X]=delta_px*i+p.tx;
        trajy_table[p.num_talbe][i+flight_points][Z]=delta_pz*i+p.tz;
    }

}

void gait_loop()
{
    int ps[4];
    for(int i=0;i<period_points;i++)
    {
        for(int j=0;j<4;j++)
        {
            ps[j]=i+int(phase[j]*period_points);
            if(ps[j]>=period_points)
                ps[j]-=period_points;
            leg[j].set_pose(trajy_table[leg[j].table][ps[j]][X]+leg[j].spx,trajy_table[leg[j].table][ps[j]][Z]+leg[j].spz);

        }
	
		//在支撑相调整机器人(测试中)
		/*
        if((i>0.25*period_points&&i<0.5*period_points)||(i>0.75*period_points&&i<period_points))
        {
                get_rpy();
                ROLL.compute();
                PITCH.compute();
                rotation_platform(ROLL.output,PITCH.output);
                lastime=current;

                leg[LF].spz+=(-ROLL.output+PITCH.output);
                leg[RF].spz+=(ROLL.output+PITCH.output);
                leg[LB].spz+=(-ROLL.output-PITCH.output);
                leg[RB].spz+=(ROLL.output-PITCH.output);

        }
		*/

        delayMicroseconds(point_time);
    }

}

void gait_transit(uint8 type,float lift_height)
{
    //通过三段直线由当前位置过渡到步态的初始位置
    //待改进

    uint8 points=int(flight_points/3); //每段直线的插补点数
       
    float delta_x[4]; //x方向的增量
    float delta_dz[4]; //落腿时z方向的增量
    int pt[4];//点之间时间
    float delta_lz=lift_height/points;//抬腿时z方向的增量

    for(uint8 i=0;i<4;i++)
    {
        delta_x[i]=(trajy_table[leg[i].table][int(phase[i]*period_points)][X]
            +leg[i].spx-leg[i].cx)/points;
        delta_dz[i]=(trajy_table[leg[i].table][int(phase[i]*period_points)][Z]
            +leg[i].spz-leg[i].cz-lift_height)/points;
        
        pt[i]=int(10*(abs(delta_x[i])+abs(delta_dz[i])+abs(delta_lz))/(flight_speed*flight_points));
        Serial.println(pt[i]);
    }

    if(type) //walk
    {
        for(uint8 i=0;i<4;i++)
        {
            for(uint8 j=0;j<points;j++)
            {
                leg[i].mov(0,delta_lz);
                delayMicroseconds(pt[i]);
            }
            for(uint8 j=0;j<points;j++)
            {
                leg[i].mov(delta_x[i],0);
                delayMicroseconds(pt[i]);
            }
            for(uint8 j=0;j<points;j++)
            {
                leg[i].mov(0,delta_dz[i]);
                delayMicroseconds(pt[i]);
            }
            delay(200);
        }
    }   
    else //trot
    {
        for(uint8 i=0;i<points;i++)
        {
            leg[0].mov(0,delta_lz);
            leg[1].mov(0,delta_lz);
            delayMicroseconds(pt[0]);
        }
        for(uint8 i=0;i<points;i++)
        {
            leg[0].mov(delta_x[0],0);
            leg[1].mov(delta_x[1],0);
            delayMicroseconds(pt[0]);
        }
        for(uint8 i=0;i<points;i++)
        {
            leg[0].mov(0,delta_dz[0]);
            leg[1].mov(0,delta_dz[1]);
            delayMicroseconds(pt[0]);
        }
        delay(200);
        for(uint8 i=0;i<points;i++)
        {
            leg[2].mov(0,delta_lz);
            leg[3].mov(0,delta_lz);
            delayMicroseconds(pt[2]);
        }
        for(uint8 i=0;i<points;i++)
        {
            leg[2].mov(delta_x[2],0);
            leg[3].mov(delta_x[3],0);
            delayMicroseconds(pt[2]);
        }
        for(uint8 i=0;i<points;i++)
        {
            leg[2].mov(0,delta_dz[2]);
            leg[3].mov(0,delta_dz[3]);
            delayMicroseconds(pt[2]);
        }
        delay(200);
    }
      
}

void rotation_platform(float rx,float ry)
{

  leg[LF].mov(0,-rx+ry); 
  leg[RF].mov(0,rx+ry);
  leg[LB].mov(0,-rx-ry); 
  leg[RB].mov(0,rx-ry);
   
}