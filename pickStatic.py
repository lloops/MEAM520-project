#!/usr/bin/python2
from time import sleep
import numpy as np
from calculateFK_jz import calculateFK
from IKv_jz import IK_velocity
from copy import deepcopy


def pickStatic(qstart, poses, target, color, map):


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
    dobj1 = Tobj1[0:3,-1] + np.array([5, -5, -5])

    # reach point
    dreach1 = dobj1 + np.array([-70, 30, 30])

    # initial v and w
    #Vreach1 = dreach1 - d31
    Vreach1 = dreach1 - de1
    Wreach1 = np.array([0,0,0])

    q = deepcopy(qstart)
    dt = 0.1

    path = []


    lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15])
    upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30])

    # Move Gripper to reach point
    for i in range(25):
        
        dq = IK_velocity(q, Vreach1, Wreach1, 6)
        q = q + dq * dt

        qcopy = np.ravel(q)
        if(any([(qcopy[j] > upperLim[j] or qcopy[j] < lowerLim[j]) for j in range(6)])):
            break

        jpos, t0e = FK.forward(q)
        de1 = jpos[-1,:]
        Vreach1 = dreach1 - de1

        path.append(q)

    ######################################################
    # move the griper to grab target
    q_reach = np.ravel(path[-1])

    #open gripper and change orientation
    for i in range(20):
        dq2 = [0,0,0, 0, -0.0785, 1.5]
        q_reach = q_reach + dq2
        path.append(q_reach)


    # Move Gripper to Object
    q_reach = np.ravel(path[-1])
    jpos, t0e = FK.forward(q_reach)
    de1 = jpos[-1,:]
    V_grab = dobj1 - de1

    for i in range(20):

        dq = IK_velocity(q, V_grab, Wreach1, 6)
        q_reach = q_reach + dq * dt

        jpos, t0e = FK.forward(q_reach)
        de1 = jpos[-1,:]
        V_grab = dobj1 - de1

        path.append(q_reach)



    #close gripper
    q_reach =  np.ravel(path[-1])
    for i in range(20):
        dq2 = [0,0,0, 0, 0, -1.5]
        q_reach = q_reach + dq2
        path.append(q_reach)
        

    # Move back
    q_reach =  np.ravel(path[-1])
    for i in range(10):
        dq2 = [0,-0.05, 0, 0, 0, 0]
        q_reach = q_reach + dq2
        path.append(q_reach)
        

    return path
