# -*- coding: utf-8 -*-
"""
Created on Wed Nov 25 14:20:49 2020

@author: Jerry
"""

#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys
import math

from os import getcwd
sys.path.append(getcwd() + "/../Core")
from arm_controller import ArmController


import calculateFK_jz 
#from IK_velocity import IK_velocity
from IKv_jz import IK_velocity
from copy import deepcopy

#change this time coefficent if needed
#jz: real time factor ~= 0.1  
tc = 1


def pickStatic(qstart, pose, name, color, map):

    isReach = True

#1. get T_frame 0/world frame WRT frame 1/base frame (of blue/red) 
    if (str(color).lower() == 'blue'):
        
        T01 = np.array([[-1, 0, 0, 200],
                       [0, -1, 0, 200],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
    
    elif(str(color).lower() == 'red'):
        
        T01 = np.array([[1, 0, 0, 200],
                       [0, 1, 0, 200],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
    
    else:
        print("incorrect color input")
        return


    #
    Tobj0 = np.array(pose) # T_obj wrt frame 0/world frame

    Tobj1 = np.matmul(T01, Tobj0) # T_obj wrt frame "1"/base frame 
    #

    #get coords of object wrt base frame  
    dobj_1 = Tobj1[0:3, 3]
    #stop at 30mm above the object 
    dreach_1 = dobj_1 + np.array([-8, 12, 8])
    
    #get coords of end eff wrt base frame
    fk = calculateFK_jz.calculateFK()
    
    jointPos, T0e = fk.forward(qstart)
    de_1 = jointPos[-1,:]
    
    #lin. v of end eff, in unit vector (gives dir. to move toward the obj)
    v_e =  (dreach_1 - de_1) #/ 8 #/ np.linalg.norm(dreach_1 - de_1)  
    #w_e = np.array([np.nan,np.nan,np.nan]) #keep ang. v 0s during the linear motion
    w_e = np.array([0,0,0])
    #w_e = np.array([np.nan,np.nan,np.nan])
    
    
    isReach = False
    
    qCurr = qstart
    deCurr = de_1
    
    
    
    dt = tc * 0.1
    
    lynx = ArmController(str(color))
    sleep(1)
    
    
    cnt = 0
    
    #if ran for 200 loops still can't reach: fails
    while(cnt<230):
        
        print(cnt)
        if( checkReach(deCurr, dreach_1) ):
            isReach = True
            #stop the robot
            lynx.set_vel([0,0,0,0,0,0])
            sleep(5)
            break
        
        #find joint vel. to create lin. motion toward the obj
        dq = IK_velocity(qCurr, v_e, w_e, 6)
        
        print("------------")
        print("dq")
        print(dq)
        print()
        
        lynx.set_vel(dq.reshape(6).tolist())
        print("moving")
        sleep(dt)
        
#        lynx.set_vel([0,0,0,0,0,0])
#        print("stopping")
#        sleep(1)
        
        #update qCurr
        [qCurr, qd]  = lynx.get_state()
        
        #update deCurr
        jointPos, T0e = fk.forward(qCurr)
        deCurr = jointPos[-1,:]
        
        #update v_e
        v_e = (dreach_1 - deCurr) #/ 8 #/ np.linalg.norm(dreach_1 - deCurr)  
        
        cnt+=1
        
        
        
    if(cnt==230):
        print("Failed")
    else:
        print("within reach")
        
    lynx.set_vel([0,0,0,0,0,0])
    
#    qClose = qCurr
#    qClose[5] = 0
#    
#
#    #lynx.set_pos(qClose)
#    #sleep(5)

    [qCurr, qd]  = lynx.get_state()
    
    lynx.stop()
    return True, qCurr
        






def checkReach(deCurr, dreach_1):
    print("checking...")
    print("deCurr")
    print(deCurr)
    print("dreach_1")
    print(dreach_1)
    print()
    print("x_e - x_reach")
    print(deCurr[0] - dreach_1[0])
    print("y_e - y_reach")
    print(deCurr[1] - dreach_1[1])
    print("z_e - z_reach")
    print(deCurr[2] - dreach_1[2])
    print()
    
    return (
            
            (abs(deCurr[0] - dreach_1[0]) <= 20) 
            and  (abs(deCurr[1] - dreach_1[1]) <= 20)
            and  ( abs(deCurr[2] - dreach_1[2]) <= 10)
            
            )        