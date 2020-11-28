#!/usr/bin/python2
from time import sleep
import numpy as np
from calculateFK_jz import calculateFK
from IKv_jz import IK_velocity
from calculateIK import calculateIK
from copy import deepcopy


def pickStatic(lynx, qstart, poses, target, color):


    isReach = True
    ########################################################################
    # Transfer target pose to lynx base frame
    ########################################################################
    # get transformation matrix from world to base frame
    #T01 = np.zeros((4,4))

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
        return


    # get transformation matrix from object to base frame

    Tobj0 = np.array(poses[target]) #target object in world frame

    Tobj1 = np.matmul(T01, Tobj0) #target object in base frame

    FK = calculateFK()

    jpos, t0e = FK.forward(qstart)

    # end effector position
    de1 = jpos[-1,:]

    # wrist position
    d31 = jpos[3,:]

    # target position
    dobj1 = Tobj1[0:3,-1] + np.array([5, -5, -8])

    # reach point
    dreach1 = dobj1 + np.array([-50, 25, 50])


    ###########################################################
    # IK Method
    ###########################################################
    # Angle btw X_base axis and drop location
    cos_theta = ((np.array([1,0,0]).dot(dreach1)) / np.linalg.norm(dreach1))
    theta = np.arccos(cos_theta)

    Te1 = np.array([[np.sin(theta), 0, cos_theta, dreach1[0]],
                    [cos_theta, 0, -np.sin(theta), dreach1[1]],
                    [0, 1, 0, dreach1[2]],
                    [0, 0, 0, 1]])

    IK = calculateIK()

    q_reach_IK, isPos = IK.inverse(Te1)

    q_reach_IK = np.append(np.ravel(q_reach_IK)[range(4)], [0,0])

    q = deepcopy(qstart)

    path = []

    path.append(np.ravel(q))

    path.append(q_reach_IK)

    #####################################################################


    #####################################################
    # move the griper to grab target
    q_reach = np.ravel(path[-1])

    #open gripper and change orientation
    while (q_reach[-1] < 30 or q_reach[-2] > -1.57):
        q_reach = np.ravel(path[-1])

        if(q_reach[-1] < 30 and q_reach[-2] > -1.57):
            dq2 = [0,0,0, 0, -0.0785, 1.5]
            q_reach = q_reach + dq2
            path.append(q_reach)
        elif(q_reach[-1] < 30):
            q_reach = q_reach + [0,0,0, 0, 0, 1.5]
            path.append(q_reach)
        elif(q_reach[-2] > -1.57):
            q_reach = q_reach + [0,0,0, 0, -0.0785, 0]
            path.append(q_reach)
        else:
            q_reach[-1] = 30
            q_reach[-2] = -1.57
            path.append(q_reach)
            break


    # Move Gripper to Object (Velocity IK)
    q_reach = np.ravel(path[-1])
    jpos, t0e = FK.forward(q_reach)
    de1 = jpos[-1,:]
    V_grab = dobj1 - de1
    Wreach1 = np.array([0,0,0])
    dt = 0.1

    loop_counter = 0
    
    for i in range(20):

        dq = IK_velocity(q_reach, V_grab, Wreach1, 6)
        q_reach = q_reach + dq * dt

        jpos, t0e = FK.forward(q_reach)
        de1 = jpos[-1,:]
        V_grab = dobj1 - de1

        path.append(q_reach)
        loop_counter += 1

        # if(np.linalg.norm(dq[0:4]) < 0.05 or loop_counter > 20):
        #     break



    #close gripper
    q_reach =  np.ravel(path[-1])
    for i in range(10):
        dq2 = [0,0,0, 0, 0, -3]
        q_reach = q_reach + dq2
        path.append(q_reach)
        

    # Move back
    q_reach =  np.ravel(path[-1])
    for i in range(5):
        dq2 = [0,-0.1, 0, 0, 0, 0]
        q_reach = q_reach + dq2
        path.append(q_reach)

    return path
