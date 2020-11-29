#!/usr/bin/python2
from time import sleep
import numpy as np
from calculateFK import calculateFK
from IKv import IK_velocity
from calculateIK import calculateIK
from copy import deepcopy
from time import sleep
import operator


def drop(qstart, color, pose):
    """
    This function plans a path to drop objects to a target location
    :param qstart:      initial pose of the robot (1x6).
    :param color:      string of color of the robot we are using (blue or red)
    :param pose:      (1x3) the location in world frame of the highest object on green plate
                        picked up, then it will try to stack it on top of the first)
    :return:
            path - Nx6 path until the object is dropped
    """

    #############################################
    # Compute transformation matrix of drop pose
    #############################################

    # Height and dropping location
    if(len(pose) == 0):
        # Case no object on green plate, drop to pre-defined location
        height = 60.0 #+ 22.0
        d_drop0_blue = np.array([100., 500., height, 1]).reshape((4,1))
        d_drop0_red = np.array([-100., -500., height, 1]).reshape((4,1))

    else:
        # Else stack onto the highest cube
        height = pose[-1] + 28.0
        d_drop0_blue = np.array([pose[0], pose[1], height, 1]).reshape((4,1))
        d_drop0_red = np.array([pose[0], pose[1], height, 1]).reshape((4,1))


    if (str(color).lower() == 'blue'):

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

    #path.append(qstart)

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

    release_count = 1
    while (q_release[-1] < 25):

        dq2 = [0,0,0, 0, 0, 5]
        dq2[-1] += release_count*2
        q_release = q_release + dq2
        path.append(q_release)
        release_count += 1


    q_release[1] = q_drop[1] - 0.3

    path.append(q_release)

    return path

##########################################################################
# Control Lynx to drop

def drop_object(lynx, color):

    qStart, vel = lynx.get_state()

    # get pose of all cubes on green plate
    [name, pose, twist] = lynx.get_object_state()

    if(color == "blue"):
        list = [[pose[i], pose[i][2,3]] for i in range(len(name)) if (pose[i][1,3]>0 and pose[i][1,3] > 440 and pose[i][2,3]>0) ]
    else:
        list = [ [pose[i], pose[i][2,3]] for i in range(len(name)) if (pose[i][1,3]<0 and pose[i][1,3] < -440 and pose[i][2,3]>0)]

    # Get path of dropping object
    if(len(list) == 0):
        path = drop(qStart, color, [])

    else:

        list = sorted(list, key=operator.itemgetter(1))

        target_pose = list[-1][0]

        target_loc = target_pose[0:3,-1]

        path = drop(qStart, color, target_loc)
        

    # iterate over target waypoints
    for p in path:
        print("Goal:")
        print(p)

        lynx.set_pos(np.ravel(p))

        while True:
            sleep(0.05)
            pos, vel = lynx.get_state()

            if np.linalg.norm(np.array(pos[0:4])- np.array(p[0:4])) <= 0.02:
                break
