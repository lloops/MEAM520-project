#!/usr/bin/python2
from time import sleep
import numpy as np
from calculateIK import calculateIK


def pickStatic(qstart, poses, target, color, map):
    """
    This function plans a path to pick static objects
    :param qstart:      initial pose of the robot (1x6).
    :param map:         the map struct
    :param poses:       all poses of the objects (lists of transformation matrices T_obj^0)
    :param target:      index of poses of the target object (int)
    :param color:      string of color of the robot we are using (blue or red)
    :return:
            path - Nx6 path until the object is picked up
            isReach - True if reachable, False if not reachable
    """

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

    ############################################################################
    # Adjust orientation of end effector to face -z axis (white face)
    ############################################################################
    Teobj = np.zeros((4,4)) #e in object frame, (make 10 mm from the target z axis to offset the cube dimension (20x20x20))

    if(abs(Tobj0[2,2] - 1) <= 0.1): #case the z axis of target cube is facing up
        #adjust gripper to reach from top down
        Teobj = np.array([[1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, -1, 10],
                        [0, 0, 0, 1]])

    elif(abs(Tobj0[2,2] + 1) <= 0.1): #case z axis of target is facing down
        #adjust gripper to reach from top down
        Teobj = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, -10],
                        [0, 0, 0, 1]])

    else: #z axis of target cube is facing left or right in horizontal

        if(abs(Tobj0[0,2]) - 1 <= 0.1): #case x axis of target is facing up or down
            #adjust gripper to reach from left to right
            Teobj = np.array([[0, -1, 0, 0],
                            [-1, 0, 0, 0],
                            [0, 0, -1, 10],
                            [0, 0, 0, 1]])

        else:
            #adjust gripper to reach from left to right
            Teobj = np.array([[1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, -1, 10],
                        [0, 0, 0, 1]])


    Te1 = np.matmul(Tobj1, Teobj) #this give the picking up pose of e wrt base frame

    #define another pose a distance away from object z axis as reaching point
    #so that path to the reaching point will not collide with the target object
    T_reach_e = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, -20],
                        [0, 0, 0, 1]]) #move e back 20mm in Ze direction

    Treach1 = np.matmul(Te1, T_reach_e) #pose of the reaching point


    ############################################################################
    # Use IK to compute lynx config (q) of the picking up pose and reach pose
    ############################################################################
    IK = calculateIK()

    q_reach, isPos = IK.inverse(Treach1)
    q_target, isPos2 = IK.inverse(Te1)

    if(isPos == 0):
        # aj = np.array([[1, 0, 0, 0],
        #                 [0, -1, 0, 0],
        #                 [0, 0, -1, 20],
        #                 [0, 0, 0, 1]]) #try gripper approach from the opposite side

        print("not feasible")
        #Te1 = np.matmul(Te1, aj)
        Te1 = np.array([[0, 0, 1, Tobj1[0,-1]-10],
                        [1, 0, 0, Tobj1[1,-1]],
                        [0, 1, 0, Tobj1[2,-1]],
                        [0, 0, 0, 1]]) #move e back 20mm in Ze direction

        Treach1 = np.matmul(Te1, T_reach_e)
        q_reach, isPos = IK.inverse(Treach1)
        q_target, isPos2 = IK.inverse(Te1)

    elif(isPos == 2): #case outside workspace, opponent's objects
        print("opponent's objects")
        isReach = False
        return q_target, q_reach, isReach



    #set gripper to fully open (30)
    q_reach = np.append(np.ravel(q_reach)[range(5)],30)
    q_target =  np.append(np.ravel(q_target)[range(5)],30)
    # print("Treach1")
    # print(Treach1)
    # print("Tobj1")
    # print(Tobj1)
    # print("Te1")
    # print(Te1)
    return q_target, q_reach, isReach
