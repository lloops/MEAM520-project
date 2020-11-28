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
from drop import drop_object
import operator

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
    #
    # picked_count = 0
    # while pickDynamic(lynx, color, picked_count):
    #     picked_count += 1
    #     continue
    #
    # lynx.stop()

    #############################################
    # Testing Static
    #############################################
    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()
    #color = sys.argv[1]
    map_struct = loadmap("maps/final.txt")
    q_start = [0, 0, 0, 0, 0, 0]

    [name, pose, twist] = lynx.get_object_state()

    #extract static, endorsed from pickDynamic()
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

    #1. if the object's name is static;
    #2. FOR BLUE: if the y coord of the obj is > 0 (-y coord are opponent's)
    #3. sometimes when simulation is initalized., a block may get struck off from the platform, which would fall to z coord (world frame) ~= -999
    if(color == "blue"):
        static_lst = [ [i] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]>0 and pose[i][2,3]>0) ]

    else:
        static_lst = [ [i] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]<0 and pose[i][2,3]>0) ]



    for j in range(len(static_lst)):
        Tobj_1 = T01.dot(pose[static_lst[j][0]])
        dist_1 = np.linalg.norm(Tobj_1[0:3,3])
        static_lst[j].append(dist_1)

    #sort the static cube from nearest dist. to base frame to furthest
    static_lst = sorted(static_lst, key=operator.itemgetter(1))


    # Update q from last dropped pose
    q_iter = q_start
    for i in range(len(static_lst)):
        print(" ")
        print(i)

        [name, pose, twist] = lynx.get_object_state()
        path = pickStatic(lynx, q_iter, pose, static_lst[i][0], color)
        print("path")
        
        lynx.set_pos(q_iter)
        sleep(1)
        
        path_counter = 0
        for p in path:
            print("Goal:")
            print(p)

            lynx.set_pos(np.ravel(p))

            if(path_counter > 1):
                sleep(0.15)
            elif(path_counter == 0):
                sleep(0.5)
            else:
                sleep(5)

            path_counter += 1
            

        path_drop = drop_object(lynx, path[-1], color, i)

        q_iter = np.ravel(path_drop[-1])


    lynx.stop()

    #############################################
    # Testing drop Only
    #############################################
    # map_struct = loadmap("maps/final.txt")
    # q_start = [-0.5, 0.5, 0.1, 0, 0, 0]
    #
    # count = 0
    #
    # lynx.set_pos(q_start)
    # sleep(5)
    #
    # # for p in path:
    # #     print(p)
    # #     lynx.set_pos(np.ravel(p))
    # #     sleep(5)
    #
    #
    # drop_object(lynx, q_start, color, count)
    #
    # lynx.stop()



    # get state of your opponent's robot
# [q, qd]  = lynx.get_opponent_state()
# print(q)
# print(qd)
