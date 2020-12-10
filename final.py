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

    # Wait for Start Gun to be fired
    lynx.wait_for_start()

    #############################################
    # Main Code
    #############################################

    while True:

        # value indicates whether the dynamic function has picked an object  1 = yes, 0 = no
        # dyn_target_ind indicates the id of that dynamic object picked, id = -1 if value = 0
        value, dyn_target_ind = pickDynamic(lynx, color)

        if value == 1:
            # call drop function to drop the dynamic object picked
            drop_object(lynx, color, dyn_target_ind, value)
        
        else:
            # no dynamic object to pick / wait too long to pick a dynamic object
            # pick a static object instead
            staticMover = pickStatic(lynx, color)

            # target_index = id of the static object picked
            target_index = staticMover.pick()

            # we have successfully picked a static object
            if target_index != -1:
                drop_object(lynx, color, target_index, 2)

            # no more statics objects left to pick
            else:
                print("no more static objects")

    lynx.stop()
