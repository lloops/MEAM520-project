#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys
import math

from os import getcwd
sys.path.append(getcwd() + "/../Core")
from arm_controller import ArmController
from loadmap import loadmap
from pickStatic_jz import pickStatic

import operator

if __name__=='__main__':

    color = sys.argv[1]
    lynx = ArmController(str(color))
    sleep(1)


    #############################################
    # Testing Static
    #############################################
    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()
        
    #color = sys.argv[1]
    map_struct = loadmap("maps/final.txt")
    q_start = np.array([-0.70, -0., 0, 0.1, 1.5, 30])
    
    lynx.set_pos(q_start)
    #adjustable, depending on rtf
    sleep(20)

    [name, pose, twist] = lynx.get_object_state()


##
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
##
        
        
        
        
    #1. if the object's name is static; 
    #2. FOR BLUE: if the y coord of the obj is > 0 (-y coord are opponent's)
    #3. sometimes when simulation is initalized., a block may get struck off from the platform, which would fall to z coord (world frame) ~= -999
    if(color == "blue"):
        static_lst = [ [name[i], pose[i], twist[i]] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]>0 and pose[i][2,3]>0) ]
        
    else:
        static_lst = [ [name[i], pose[i], twist[i]] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]<0 and pose[i][2,3]>0) ]



    for j in range(len(static_lst)):
        Tobj_1 = T01.dot(pose[j])
        dist_1 = np.linalg.norm(Tobj_1[0:3,3])
        static_lst[j].append(dist_1)
    
    #sort the static cube from nearest dist. to base frame to furthest
    static_lst = sorted(static_lst, key=operator.itemgetter(3))

    print("static_lst")
    print(static_lst)
    
    
    for i in range(len(static_lst)):
    #test pickStatic on the nearest obj (wrt. base frame):
        obj = static_lst[i]
        #test on only 1 static object
        print("= = = = = =")
        print("name: "+str(obj[0]))
#        print("pose: "+str(obj[1]))
#        print("twist: "+str(obj[2]))
        print("= = = = = =")
        
        name = obj[0]
        pose = obj[1]
        
        [qCurr, qd]  = lynx.get_state()
        q_start = qCurr
        #q_target, q_reach, isReach = pickStatic(q_start, pose, name, color, map_struct)
        
        #TODO: update the static objects status after each run; perhaps not to use the current for loop structure 
        res, qCurr = pickStatic(q_start, pose, name, color, map_struct)
        
        
        qClose = qCurr
        qClose[5] = 0
        lynx.set_pos(qClose)
        sleep(5)
    
        qBack = [-0.70, 0, 0, 0, qClose[4], 0]
        lynx.set_pos(qBack)
        sleep(10)
        
        q_start = np.array([-0.70, -0., 0, 0.1, 1.5, 30])
        lynx.set_pos(q_start)
        #adjustable, depending on rtf
        sleep(5)
        
    lynx.stop()
    
    print("static_lst")
    print(static_lst)
    # get state of your opponent's robot
# [q, qd]  = lynx.get_opponent_state()
# print(q)
# print(qd)
