#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys
import math

from os import getcwd
sys.path.append(getcwd() + "/../Core")
from arm_controller import ArmController

from pickDynamic import pickDynamic
from loadmap import loadmap
from pickStatic import pickStatic
from pickStatic0 import pickStatic0

if __name__=='__main__':

    color = sys.argv[1]
    lynx = ArmController(str(color))
    sleep(1)

    #############################################
    # Testing Dynamic
    #############################################
    # q_start = np.array([0,  0, 0, 0, 0, 0])
    # lynx.set_pos(q_start)
    # sleep(3)
    # while pickDynamic(lynx):
    #     continue

    # lynx.stop()

    #############################################
    # Testing Static
    #############################################
    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()
    #color = sys.argv[1]
    map_struct = loadmap("maps/final.txt")
    q_start = np.array([0,  0, 0, 0, 0, 0])
    
    [name, pose, twist] = lynx.get_object_state()
    
    #extract static, endorsed from pickDynamic()
    statics = []
    for i in range(13):
        if name[i][5]=="s":
            statics.append(i)
    
    # print(statics)
    # print(name)
    #print(pose[statics[0]])
    for i in range(8):
        print(" ")
        print(i)
        q_target, q_reach, isReach = pickStatic(q_start, pose, statics[i], color, map_struct)
        #pickStatic(q_start, pose, statics[i], color, map_struct)
    
        # print(q_target)
        # print(q_reach)
        if(isReach):
            lynx.set_pos(q_start)
            sleep(5)
            path = pickStatic0(q_start, pose, statics[i], color, map_struct)
            for p in path:
    
                lynx.set_pos(p)
                sleep(0.5)
    lynx.stop()


    # get state of your opponent's robot
# [q, qd]  = lynx.get_opponent_state()
# print(q)
# print(qd)
