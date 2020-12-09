
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

    #lynx.wait_for_start() # Wait for Start Gun to be fired

    #############################################
    # Main Code
    #############################################

    while True:
        value, dyn_target_ind = pickDynamic(lynx, color)

        if value == 1:
            drop_object(lynx, color, dyn_target_ind, value)
        else:
            staticMover = pickStatic(lynx, color)
            target_index = staticMover.pick()

            # if there is no more statics, continue to wait for dynamic
            if target_index != -1:
                drop_object(lynx, color, target_index, 2)
            else:
                print("no more static objects")

    lynx.stop()
