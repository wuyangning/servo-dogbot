# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 16:27:27 2020

@author: YANG
"""
import matplotlib.pyplot as plt
import math as m
import time

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
point = [[0,0],[X1,Y1],[0,0],[0,0],[0,0],[0,-80.0]] #绝对坐标

draw_link = False

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


#同时等速匀速输出两个关节
def mov(cx,cy,tx,ty):
    
    global theta0
    global theta1
    global point
    
    point[5][X]=cx
    point[5][Y]=cy
    IK()
    current_theta0=theta0
    current_theta1=theta1
    
    point[5][X]=tx
    point[5][Y]=ty
    IK()
    target_theta0=theta0
    target_theta1=theta1
 
    delta0=target_theta0-current_theta0
    delta1=target_theta1-current_theta1
    max_delta=max(abs(delta0),abs(delta1))
    steps=int(max_delta/0.02)

    for i in range(steps):
        if abs(current_theta0-target_theta0)>0.02:
            if delta0>0:
                current_theta0+=0.02
            else:
                current_theta0-=0.02
        if abs(current_theta1-target_theta1)>0.02:
            if delta1>0:
                current_theta1+=0.02
            else:
                current_theta1-=0.02
        
        theta0=current_theta0
        theta1=current_theta1
        
        FK()
        
        if draw_link:
            plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps,1])
            plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps,1])
            plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps,1])
            plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps,1])
    
        plt.scatter(point[5][X],point[5][Y],color=[0,i/steps,1],s=2)


#先后转动两个关节
#order=1先转theta0
def mov1(cx,cy,tx,ty,order):
    
    global theta0
    global theta1
    global point
    
    point[5][X]=cx
    point[5][Y]=cy
    IK()
    current_theta0=theta0
    current_theta1=theta1
    
    point[5][X]=tx
    point[5][Y]=ty
    IK()
    target_theta0=theta0
    target_theta1=theta1
 
    delta0=target_theta0-current_theta0
    delta1=target_theta1-current_theta1
    steps0=int(abs(delta0/0.02))
    steps1=int(abs(delta1/0.02))
    
    if order:
        for i in range(steps0):
            if abs(current_theta0-target_theta0)>0.02:
                if delta0>0:
                    current_theta0+=0.02
                else:
                    current_theta0-=0.02
            theta0=current_theta0
            theta1=current_theta1
    
            FK()
            
            if draw_link:
                plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[1,i/steps0,0])
                plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[1,i/steps0,0])
                plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[1,i/steps0,0])
                plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[1,i/steps0,0])
        
            plt.scatter(point[5][X],point[5][Y],color=[1,i/steps0,0],s=2)
        
        
        for i in range(steps1):
            if abs(current_theta1-target_theta1)>0.02:
                if delta1>0:
                    current_theta1+=0.02
                else:
                    current_theta1-=0.02
            theta1=current_theta1
    
            FK()
            
            if draw_link:
                plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps1,1])
                plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps1,1])
                plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps1,1])
                plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps1,1])
        
            plt.scatter(point[5][X],point[5][Y],color=[0,i/steps1,1],s=2)
    else:
        
        for i in range(steps1):
            if abs(current_theta1-target_theta1)>0.02:
                if delta1>0:
                    current_theta1+=0.02
                else:
                    current_theta1-=0.02
            theta0=current_theta0
            theta1=current_theta1
    
            FK()
            
            if draw_link:
                plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps1,1])
                plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps1,1])
                plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps1,1])
                plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps1,1])
        
            plt.scatter(point[5][X],point[5][Y],color=[0,i/steps1,1],s=2)
        
        
        for i in range(steps0):
            if abs(current_theta0-target_theta0)>0.02:
                if delta0>0:
                    current_theta0+=0.02
                else:
                    current_theta0-=0.02
            theta0=current_theta0
    
            FK()
            
            if draw_link:
                plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[1,i/steps0,0])
                plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[1,i/steps0,0])
                plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[1,i/steps0,0])
                plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[1,i/steps0,0])
        
            plt.scatter(point[5][X],point[5][Y],color=[1,i/steps0,0],s=2)
            
#同时不等速匀速输出两个关节          
def mov2(cx,cy,tx,ty):
    
    global theta0
    global theta1
    global point
    
    point[5][X]=cx
    point[5][Y]=cy
    IK()
    current_theta0=theta0
    current_theta1=theta1
    
    point[5][X]=tx
    point[5][Y]=ty
    IK()
    target_theta0=theta0
    target_theta1=theta1
    
    steps=30
 
    delta0=target_theta0-current_theta0
    delta1=target_theta1-current_theta1
    inc0=delta0/steps
    inc1=delta1/steps

    for i in range(steps):
        current_theta0+=inc0
        current_theta1+=inc1
        
        theta0=current_theta0
        theta1=current_theta1
        
        FK()
        
        if draw_link:
            plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps,1])
            plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps,1])
            plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps,1])
            plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps,1])
    
        plt.scatter(point[5][X],point[5][Y],color=[0,i/steps,1],s=2)


#四点规划          
def mov3(cx,cy,tx,ty,hx,hy,lx,ly):
    
    global theta0
    global theta1
    global point
    start = time.clock()
    point[5][X]=cx
    point[5][Y]=cy
    
    fsteps=40 #抬腿到落腿的轨迹点数
    bsteps=20 #收腿的轨迹点数
    steps=100
    
    ch_steps=abs(int((hx-cx)*fsteps/(tx-cx)))
    ht_steps=fsteps-ch_steps
    tl_steps=abs(int((lx-tx)*bsteps/(tx-cx)))
    lc_steps=bsteps-tl_steps
    
    inc_fx=(tx-cx)/fsteps
    inc_bx=(tx-cx)/bsteps
    
    inc_chy=(hy-cy)/ch_steps
    inc_hty=(ty-hy)/ht_steps
    inc_tly=(ly-ty)/tl_steps
    inc_lcy=(cy-ly)/lc_steps
    
    

    for i in range(ch_steps):
        
        point[5][X]+=inc_fx
        point[5][Y]+=inc_chy
        
        IK()
        steps-=1
        if draw_link:
            plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps,1])
            plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps,1])
            plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps,1])
            plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps,1])
    
        plt.scatter(point[5][X],point[5][Y],color=[0,i/steps,1],s=2)

        
        
    for i in range(ht_steps):
        
        point[5][X]+=inc_fx
        point[5][Y]+=inc_hty
        
        IK()
        steps-=1
        if draw_link:
            plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps,1])
            plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps,1])
            plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps,1])
            plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps,1])
    
        plt.scatter(point[5][X],point[5][Y],color=[0,i/steps,1],s=2)
        
        
    for i in range(tl_steps):
        
        point[5][X]-=inc_bx
        point[5][Y]+=inc_tly
        
        IK()
        steps-=1
        if draw_link:
            plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps,1])
            plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps,1])
            plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps,1])
            plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps,1])
    
        plt.scatter(point[5][X],point[5][Y],color=[0,i/steps,1],s=2)
        
        
    for i in range(lc_steps):
        
        point[5][X]-=inc_bx
        point[5][Y]+=inc_lcy
        
        IK()
        steps-=1
        if draw_link:
            plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps,1])
            plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps,1])
            plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps,1])
            plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps,1])
    
        plt.scatter(point[5][X],point[5][Y],color=[0,i/steps,1],s=2)
    print(time.clock()-start)


#双半椭圆规划
def mov4(cx,cy,tx,ty,H,h):
    
    global theta0
    global theta1
    global point
    
    
    fsteps=10 #抬腿到落腿的轨迹点数
    bsteps=10 #收腿的轨迹点数
    
    steps=fsteps+bsteps+40
    
    phi=m.atan2(ty-cy,tx-cx)-m.pi
    if phi>=m.pi:
        phi=0
    print(phi)
    dx=(cx+tx)/2
    dy=(cy+ty)/2
    
    d=m.sqrt(m.pow(ty-cy,2)+m.pow(tx-cx,2))
    
    

    for i in range(fsteps):
        
        theta=i*m.pi/fsteps
        
        point[5][X]=0.5*d*m.cos(theta)*m.cos(phi)-0.5*H*m.sin(theta)*m.sin(phi)+dx
        point[5][Y]=0.5*d*m.cos(theta)*m.sin(phi)+0.5*H*m.sin(theta)*m.cos(phi)+dy
        
        IK()
        steps-=1
        if draw_link:
            plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps,0.2])
            plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps,0.2])
            plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps,0.2])
            plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps,0.2])
    
        plt.scatter(point[5][X],point[5][Y],color=[1,i/steps,1],s=2*i)


    for i in range(bsteps):
        
        theta=m.pi+i*m.pi/bsteps
        
        point[5][X]=0.5*d*m.cos(theta)*m.cos(phi)-0.5*h*m.sin(theta)*m.sin(phi)+dx
        point[5][Y]=0.5*d*m.cos(theta)*m.sin(phi)+0.5*h*m.sin(theta)*m.cos(phi)+dy
        
        IK()
        steps-=1
        if draw_link:
            plt.plot([point[1][X],point[2][X]], [point[1][Y],point[2][Y]], color=[0,i/steps,1,0.2])
            plt.plot([point[0][X],point[4][X]], [point[0][Y],point[4][Y]], color=[0,i/steps,1,0.2])
            plt.plot([point[2][X],point[3][X]], [point[2][Y],point[3][Y]], color=[0,i/steps,1,0.2])
            plt.plot([point[3][X],point[5][X]], [point[3][Y],point[5][Y]], color=[0,i/steps,1,0.2])
    
        plt.scatter(point[5][X],point[5][Y],color=[1,i/steps,1],s=i)


        
        
    
    
if __name__ == '__main__':
    
    plt.figure(figsize=(12,12))
    plt.xlim(-80, 100)
    plt.ylim(-100, 80)
    
    #draw_link=True
    
    #mov3(25,-75, -25,-75,  20,-60, -20,-80)
    mov4(25,-75,-25,-75,20,5)




   
    

    
   
    
   

    
    

    