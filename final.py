#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys

from os import getcwd
sys.path.append(getcwd() + "/../Core")
from arm_controller import ArmController

from pickDynamic import pickDynamic
from loadmap import loadmap
from pickStatic import pickStatic
from drop import drop_object

if __name__=='__main__':

    map_struct = loadmap("maps/final.txt")

    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()

    color = str(sys.argv[1])
    lynx = ArmController(str(color))
    sleep(1)

    #############################################
    # Main Code
    #############################################
    
    # staticMover = pickStatic(lynx, color)
    # staticMover.pick()
    # drop_object(lynx, color, 0)

    # staticMover = pickStatic(lynx, color)
    # staticMover.pick()
    # drop_object(lynx, color, 1)

    pickDynamic(lynx, color)
    drop_object(lynx, color, 0)

    pickDynamic(lynx, color)
    drop_object(lynx, color, 1)

    pickDynamic(lynx, color)
    drop_object(lynx, color, 2)

    # staticMover = pickStatic(lynx, color)
    # staticMover.pick()
    # drop_object(lynx, color, 4)
    
    lynx.stop()


# get state of your opponent's robot
# [q, qd]  = lynx.get_opponent_state()
# print(q)
# print(qd)
