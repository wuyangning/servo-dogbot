# 简介
这个项目是基于T站上的一个模仿波士顿动力的小黄狗的模型。原作者把他叫做spotmicro  （https://www.thingiverse.com/thing:3445283） 用的是996R的舵机。

虽然结构很好看，但是缺陷比较多，加上树莓派电池之后重量差不多在2.5kg左右，996舵机只能勉强能撑起这只狗。
这个机器狗也只能算是半成品，我只能分享一下我做的制作过程中的一些思路，并不能提供一个完整的机器狗控制程序。这也是我第一次做足式机器人，文献看得不多，有什么不对的地方欢迎指正，一起交流。

# 结构和硬件
按照原作者的模型需要：

- 12个MG996R舵机（应该是最廉价的舵机了）
- 8个F625法兰轴承（内径有点小，设计成mr128轴承会更好）
- 8个M5*15螺栓
- 40个M4*20螺栓
- 8个M4*15螺栓
- 48个M4螺母
- 4个M3*20螺栓
- 28个M3*10螺栓
- 16个M3螺母

然后我把控制部分全换了：

- 树莓派
- stm32f103c8t6
- 自带卡尔曼滤波的9轴陀螺仪（省事）
- pca9685pwm扩展板
- 5v10A-20A稳压
- usb摄像头
- 3s或2s电池


# 程序

运动控制部分基于arduino_stm32,为了方便开发使用里面的FreeRTOS，参考：
> http://docs.leaflabs.com/static.leaflabs.com/pub/leaflabs/maple-docs/latest/maple-quickstart.html


## 指令
指令采用“起始符+数字 字符+数字 字符+数字+...+结束符”的形式，例如“M1 X10 Y10 \n”。"M1"指定了运动控制的类型，前进或者是转弯；中间部分的"X10 Y10"指定了一些必要的参数，向前多少距离或者是转弯多少度；最后“\n”结束符表示这条指令结束。这个类似于数控系统的G代码。也可以用16进制的协议，不过字符的会更直观一点。

## 单腿运动学
单只腿只有三个自由度，所以这里用几何法来解运动学正逆解。以左边的腿为例。如图所示，简化关节后构建右手坐标系（右边的腿的话也可以用左边的正逆解，使用的时候把y取反就行了）。  

#### 定义关节角
如图分别是左腿的前视图、左腿的大腿和小腿所平面视图。定义髋关节的关节角为$\theta_0$,大腿关节为$\theta_1$,小腿关节为$\theta_2$。大腿长l1，小腿长l2，腿部平面和髋关节的偏置为d。
![image](http://wx3.sinaimg.cn/mw690/e8d4eb99gy1g2h3swh4z1j20v70k0mys.jpg)
#### 运动学正解
运动学正解在控制中暂时还没用到过，但还是先实现一下  
先看大腿和小腿所在的平面( ),定义另外一个坐标轴垂直于x轴为Rz：

$$

x=l_1cos\theta_1+l_2cos(\theta_1-\theta_2)

R_z=l_1sin\theta_1+l_2sin(\theta_1-\theta_2)

$$

转到前视图：

$$   
y=scos\varphi

z=ssin\varphi

s=\sqrt{d^2+R^2_z}

\varphi=\theta_0-atan2(R_z,d)
$$  



#### 运动学逆解
先看前视图：

$$ 
R_z=\sqrt{y^2+z^2-d^2}

\theta_0=atan2(z,y)+atan2(R_z,d)

$$ 

转到大腿和小腿所在的平面：  
由余弦定理得：

$$ 
x^2+R^2_z=l^2_1+l^2_2-2l_1l_2cos(\pi-\theta)

cos\theta_2=(x^2+y^2+z^2-l^2_1-l^2_2-d^2)/(2l_1l_2)

sin\theta_2=\sqrt{1-cos^2\theta_2}

\theta_2=atan2(sin\theta_2,cos\theta_2)
$$ 

作过小腿末端点的垂线交于大腿（或其延长线）：

$$ 
\theta_1=antan2(l_2sin\theta_2,l_1+l2cos\theta_2)+atan2(R_z,x)
$$ 
#### 代码
在使用运动学的时候，使用增量形式的控制。即每一次控制都让小腿末端运动一个位移增量（矢量）。

```
void KINE::mov(float vx,float vy,float vz)
{
  pose[X]+=vx;
  pose[Y]+=vy;
  pose[Z]+=vz;
  IK();
  move_servo();
}
```

## 平衡控制
机器狗平台实际上就是一个并联平台，可以完成6个自由度的运动，而且还存在冗余。不过在实际使用过程中可以不用对这个并联平台做精确的解算，只实现必要的几个特殊运动，如三个坐标的平动和身体姿态的改变就可以了。

#### 三个坐标的平动
在建立好单腿运动学模型的条件下，这个很好实现。四条腿的落脚点相对平台同时后移，平台就向前移动；同时抬起，平台就向下移动。

```
//右边的腿y是取反的
void move_platform(float x,float y,float z)//移动平台
{
    leg_lf.mov(-x,-y,-z); 
    leg_rf.mov(-x,y,-z);
    leg_lb.mov(-x,-y,-z); 
    leg_rb.mov(-x,y,-z);
}
```
#### 姿态调整
当左边的腿下放（抬起），右边的退抬起（下放）时，改变平台的roll角；当前面的腿下放（抬起），后面的退抬起（下放）时，改变平台的pitch角；四只脚按同一方向绕平台中心移动时，改变yall角。对于roll和pitch，这么做其实是不对的，因为在完整约束条件下，用这种方法除了会改变roll和pitch角，还会使整个平台向一侧偏移。
但小角度下可近似。

```
//把roll和pitch叠加在一起
//yall暂时没写
void rotation_platform(float rx,float ry)
{

  leg_lf.mov(0,0,rx-ry); 
  leg_rf.mov(0,0,-rx-ry);
  leg_lb.mov(0,0,rx+ry); 
  leg_rb.mov(0,0,-rx+ry);
   
}
```
#### 平衡
有了前面的基础，平衡控制就很简单了，用两个pid（pd控制就够了）分别控制roll和pitch角就可以了。

```
//循环
current=xTaskGetTickCount();
if((current-lastime)>=SAMEPLE_TIME)
{
    get_rpy();
    ROLL.compute();
    PITCH.compute();
    rotation_platform(-add_roll,-add_pitch);
    lastime=current;
}
```

## 步态
主流的步态调节使用cpg算法，一种用数学模型模拟中枢神经生成节律的方法。然而我不会。所以就先死调了一个步态先试试。
#### 动态步态
这里我没有使用算法让机器人维持动态平衡，只是调了一个两节拍的步态，相当于机器狗的小跑，但是效果不是很理想，感觉舵机的力矩和响应速度不够。
在行走过程中腿部的运动有三个状态，抬腿，迈腿和收腿。遵循抬多少就落多少，迈多少就收多少的原则就可以了。需要注意的是，行走之前有一个起势。

```
//给定walk_x,-walk_y,walk_ro就可以实现全向移动了。
static void walk(void *pvParameters) //行走进程
{  
  for (;;) 
  {   
     vTaskDelay(1);

    if(walking)
    {
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
```

#### 静态步态 
静态步态我还没有尝试，但是思路不复杂。只要调步态的时候保证中心始终落在支撑三角形内就可以了。比如抬起右后脚的时候，让身体左前移。


