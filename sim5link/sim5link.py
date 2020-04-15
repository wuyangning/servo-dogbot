# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 16:27:27 2020

@author: YANG
"""
#为了方便显示，仿真中的x方向和机器狗实际是反的
import matplotlib.pyplot as plt
import math as m

L1 = 55.0
L2 = 20.0
L3 = 55.0
L4 = 25.0
L5 = 55.0
X1 = 10.2
Y1 = 12.5

X=0
Y=1

theta0 = -0.602563
theta1 = 1.13439
point = [[0,0],[X1,Y1],[0,0],[0,0],[0,0],[20,-80.0]] #绝对坐标

draw_link = False

#正运动学
def FK():
    
    global theta0
    global theta1
    global point
    
    point[2][X] = L2*m.cos(theta1)+point[1][X]
    point[2][Y] = L2*m.sin(theta1)+point[1][Y]

    point[4][X] = L1*m.cos(theta0)
    point[4][Y] = L1*m.sin(theta0)

    ka=2*(point[4][X]-point[2][X])*L4
    kb=2*(point[4][Y]-point[2][Y])*L4
    kc=m.pow(point[4][X]-point[2][X],2)+m.pow(point[4][Y]-point[2][Y],2)+L4*L4-L3*L3
    theta4=2.0*m.atan2(kb+m.sqrt(ka*ka+kb*kb-kc*kc),ka-kc)

    point[3][X] = L4*m.cos(theta4)+point[4][X]
    point[3][Y] = L4*m.sin(theta4)+point[4][Y]

    point[5][X] = -L5*m.cos(theta4)+point[4][X]
    point[5][Y] = -L5*m.sin(theta4)+point[4][Y]


#逆运动学
def IK(): 
    
    global theta0
    global theta1
    global point
    
    COS_O = (L1*L1+L5*L5-m.pow(point[5][X],2)-m.pow(point[5][Y],2))/(2*L1*L5)
    SIN_O = m.sqrt(1-COS_O*COS_O)
    ANG_O = m.atan2(SIN_O,COS_O)
    
    theta0 = m.atan2(point[5][Y],point[5][X])+m.atan2(L5*SIN_O,L1-L5*COS_O)
    theta4 = ANG_O+theta0
    
    point[4][X] = point[5][X]+L5*m.cos(theta4)
    point[4][Y] = point[5][Y]+L5*m.sin(theta4)

    point[3][X] = point[4][X]+L4*m.cos(theta4)
    point[3][Y] = point[4][Y]+L4*m.sin(theta4)

    COS_F=(L2*L2+L3*L3-m.pow(point[3][X]-point[1][X],2)-m.pow(point[3][Y]-point[1][Y],2))/(2*L2*L3)
    SIN_F=m.sqrt(1-COS_F*COS_F)
    #double ANG_F=atan2(SIN_F,COS_F);
    theta1=m.atan2(point[3][Y]-point[1][Y],point[3][X]-point[1][X])+m.atan2(L3*SIN_F,L2-L3*COS_F)

    point[2][X] = L2*m.cos(theta1)+point[1][X]
    point[2][Y] = L2*m.sin(theta1)+point[1][Y]


 
    
if __name__ == '__main__':
    
    plt.figure(figsize=(12,12))
    plt.xlim(-80, 100)
    plt.ylim(-100, 80)
    
    #修改pointt[5][]计算逆解
    IK()
    #或修改theta0、theta1计算正解
    #FK()
    
    plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[1,0,0])
    plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[1,0,0])
    plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[1,0,0])
    plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[1,0,0])




   
    

    
   
    
   

    
    

    