#!/usr/bin/python2
from time import sleep
import numpy as np
from calculateFK import calculateFK
from IK_velocity import IK_velocity
from copy import deepcopy


def pickStatic0(qstart, poses, target, color, map):


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

    ########################################################################
    # Use Velocity IK to update path 
    ########################################################################
    FK = calculateFK()

    jpos, t0e = FK.forward(qstart)

    de1 = jpos[-1,:] #position of end effector in base frame

    dobj1 = Tobj1[0:3,-1] #position of target object in base frame

    Vobj1 = dobj1 - de1 #linear velocity for end effector

    Wobj1 = np.array([0,0,0]) #angular velocity of end effector

    q = deepcopy(qstart)
    dt = 0.5 #time step for update

    path = []

    for i in range(10):
        dq = IK_velocity(q, Vobj1, Wobj1, 6) #compute dq
        q = q + dq * dt  #update next q

        #update position of end effector
        jpos, t0e = FK.forward(q)
        de1 = jpos[-1,:]

        #update linear velocity
        Vobj1 = dobj1 - de1

        path.append(q)

    return path
