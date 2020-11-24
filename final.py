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

if __name__=='__main__':
    lynx = ArmController('blue')
    sleep(1)
    
    while pickDynamic(lynx):
        continue

    lynx.stop()

# if len(sys.argv) < 2:
#     print('usage: python final.py <color>')
#     sys.exit()
# color = sys.argv[1]

    # get state of your opponent's robot
# [q, qd]  = lynx.get_opponent_state()
# print(q)
# print(qd)

    
