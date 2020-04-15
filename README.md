# puppy-bot
![](https://wx2.sinaimg.cn/mw690/e8d4eb99ly1gduh8t5tcdj20u01404jr.jpg)

视频演示：https://www.bilibili.com/video/BV1if4y1U7h3
# 文件目录
- cad(step)
    - step模型文件
- puppy_bot
    - 源码，使用platformio
- sim5link
    - 简易的5杆机构运动学仿真

# 关于
- 为了把电池塞进去，由原来的12个舵机改成了8个。舵机也直接由单片机驱动。
- 通过修改一些参数就可以切换各种步态，但不是所有步态都能用。这取决于舵机的性能。测试中trot步态是效果最好的。
- 目前只有一个陀螺仪，可以适应一定的地形，但是效果并不好。貌似用一些总线舵机可以可以通过电流信息实现触地检测。
- 用的是纯位置控制。目前只完成步态和一些动作的验证。完整的控制流程和一些保护机制还需要慢慢添加。仅作参考😂。大概流程如下。

![](https://wx2.sinaimg.cn/mw690/e8d4eb99ly1gduhn6wd7rj213r0hpt9w.jpg)



# 使用
- 目前是直接用pwm输出给舵机，用的陀螺仪是串口陀螺仪GY952。如果是其他舵机驱动方式或陀螺仪。需要在myimu中先实现欧拉角的输出，在myservo中实现关节角的输出。
- 舵机需要标定，这一步比较麻烦(之后弄个上位机)。舵机脉宽和关节角的之间是一个线性关系，需要在myservo.h中确定。每条腿的连杆和关节角的定义如下(关节角是连杆以绿线为起点，逆时针旋转的角度)：
![](https://wx4.sinaimg.cn/mw690/e8d4eb99ly1gduj9z8157j20yg0u0qhj.jpg)
- 参数
```C
//LF,RB,RF,LB的步态相位
float phase[4]={0,0,0.5,0.5}
//腿悬空时的平均速度(mm/us)
//太小会导致轨迹失真，太大不能保证步态
float flight_speed=0.0003; 
//轨迹参数
//有两种轨迹可选，椭圆或两段直线
struct trajy_para
{
    float flight_period; //腿腾空的周期占比
    uint8 trajy_type; //滞空的轨迹类型 1为椭圆轨迹，0为两段直线
    float tx; //腿的目标位置
    float tz;
    float hx; //抬腿的最高位置
    float hz; //如果是椭圆轨迹，表示椭圆半短轴
    uint8 num_talbe;//存入的轨迹表

};
```
- 调用或切换步态的流程
```C
//生成轨迹存入轨迹表
trajy_para param={0.5,1,40,0,20,20,0};
trajy_planning(param);
//每条腿通过三段直线轨迹由当前位置过渡到新步态的初始位置
//两个参数分别是过渡时的步态(0:trot,1:walk)和抬腿高度
gait_transit(0,20);

void loop() 
{
    //循环执行轨迹表
    gait_loop();
}


```