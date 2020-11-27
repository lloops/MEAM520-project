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

if __name__=='__main__':

    map_struct = loadmap("maps/final.txt")

    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()

    color = sys.argv[1]
    lynx = ArmController(str(color))
    sleep(1)

    #############################################
    # Testing Dynamic
    #############################################
    q_start = np.array([0, 0, 0, 0, 0, 0])
    lynx.set_pos(q_start)
    sleep(2)
    
    while True:
        while not pickDynamic(lynx, str(color)):
            continue

        q = [0,0,0,0,0,0]
        lynx.set_pos(q)
        sleep(5)

        staticMover = pickStatic(lynx, str(color))
        staticMover.pick()
        lynx.set_pos(q)
        sleep(5)
    
    lynx.stop()


# get state of your opponent's robot
# [q, qd]  = lynx.get_opponent_state()
# print(q)
# print(qd)
