#!/usr/bin/python2
from time import sleep
import numpy as np
from calculateFK_jz import calculateFK
from IKv_jz import IK_velocity
from calculateIK import calculateIK
from copy import deepcopy
from time import sleep


def drop(qstart, color, map, count):
    """
    This function plans a path to drop objects to a target location
    :param qstart:      initial pose of the robot (1x6).
    :param map:         the map struct
    :param color:      string of color of the robot we are using (blue or red)
    :param count:      (int) the count (start from 0) of object picked up (if this is the second object
                        picked up, then it will try to stack it on top of the first)
    :return:
            path - Nx6 path until the object is dropped
    """

    #############################################
    # Compute transformation matrix of drop pose
    #############################################

    # Height of dropping location
    height = 60.0 + count * 20.0


    if (str(color).lower() == 'blue'):

        d_drop0_blue = np.array([100., 500., height, 1]).reshape((4,1))

        T01 = np.array([[-1, 0, 0, 200],
                        [0, -1, 0, 200],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        d_drop1_blue = np.matmul(T01, d_drop0_blue) #drop location in base frame


        d_drop1_blue = d_drop1_blue[0:3,:]

        # Angle btw X_base axis and drop location
        cos_theta = ((np.array([1,0,0]).dot(d_drop1_blue)) / np.linalg.norm(d_drop1_blue))[0]
        theta = np.arccos(cos_theta)

        Te1 = np.array([[np.sin(theta), 0, cos_theta, d_drop1_blue[0,0]],
                        [cos_theta, 0, -np.sin(theta), d_drop1_blue[1,0]],
                        [0, 1, 0, height],
                        [0, 0, 0, 1]])

    elif(str(color).lower() == 'red'):

        d_drop0_red = np.array([-100., -500., height, 1]).reshape((4,1))

        T01 = np.array([[1, 0, 0, 200],
                       [0, 1, 0, 200],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        d_drop1_red = np.matmul(T01, d_drop0_red) #drop location in base frame


        d_drop1_red = d_drop1_red[0:3,:]

        # Angle btw X_base axis and drop location
        cos_theta = ((np.array([1,0,0]).dot(d_drop1_red)) / np.linalg.norm(d_drop1_red))[0]
        theta = np.arccos(cos_theta)

        Te1 = np.array([[np.sin(theta), 0, cos_theta, d_drop1_red[0,0]],
                        [cos_theta, 0, -np.sin(theta), d_drop1_red[1,0]],
                        [0, 1, 0, height],
                        [0, 0, 0, 1]])

    else:
        print("incorrect color input")
        return


    #############################################
    # Compute IK for Drop off pose
    #############################################

    IK = calculateIK()

    q_drop, isPos = IK.inverse(Te1)


    #############################################
    # Plan Path to Drop off pose
    #############################################

    path = []

    if(not isPos):
        print('Not feasible pose')
        return path

    path.append(q_start)
    
    # Define intermediate point 
    Tinter1 = deepcopy(Te1)

    Tinter1[0:3,-1] = np.array([Te1[0,-1] - 30, Te1[1,-1] + 50, height + 40])

    q_inter, isPos = IK.inverse(Tinter1)

    q_inter = np.append(np.ravel(q_inter)[range(5)],0)
    q_inter[-2] = -np.pi/2
    path.append(q_inter) 

    # Add drop pose
    q_drop = np.append(np.ravel(q_drop)[range(5)],0)
    q_drop[-2] = -np.pi/2
    path.append(q_drop)

    # Release and return 
    q_release = deepcopy(q_drop)
    q_release[-1] = 30
    q_release[1] = q_drop[1] - 0.5
    path.append(q_release)


    return path



##########################################################################
# Control Lynx to drop

def drop_object(lynx, qstart, color, map_struct, count):

    path = drop(qstart, color, map_struct, count)


    wait_count = 50

    # iterate over target waypoints
    for p in path:
        print("Goal:")
        print(p)

        lynx.set_pos(np.ravel(p))
    
        reached_target = False

        # Count is number of time steps waited
        count = 0
        while not reached_target:
            # Check if robot is collided then wait
            sleep(0.1)
            pos, vel = lynx.get_state()
            # iterate count and check if should send next command
            count = count + 1
            #print(np.linalg.norm(pos-p))
            if np.linalg.norm(pos-p) <= 0.12 or count > wait_count:
                reached_target = True
                count = 0

if __name__=='__main__':

    q_start = np.array([0,0,0,0,0,0])

    color = 'blue'

    map_struct = 0

    count = 0

    path = drop(q_start, color, map_struct, count)

    print(path)
