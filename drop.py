#!/usr/bin/python2
from time import sleep
import numpy as np
from calculateFK import calculateFK
from IKv import IK_velocity
from calculateIK import calculateIK
from copy import deepcopy
from time import sleep
import operator


def drop(qstart, color, pose, list_len, object_pose):
    """
    This function plans a path to drop objects to a target location
    :param qstart:      initial pose of the robot (1x6).
    :param color:       string of color of the robot we are using (blue or red)
    :param pose:        position (3 dimensional) of the previously dropped object
    :param list_len:    Number of objects already on the dropping area
    :param object_pose: pose (4x4) of the current dropping object

    :return:
            path - Nx6 path until the object is dropped
    """

    #############################################
    # Compute transformation matrix of drop pose
    #############################################

    # Initialize FK and offsets between Gripper center and object center (currently grabbed by the gripper)
    FK = calculateFK()
    X_offset = 0
    Y_offset = 0

    # 3 drop points p1, p2, p3; (or drop "areas" since the object will not land at exactly the same x,y)
    # stack a max of 3 objects at p1, stack a max of 2 objects at p2, keep stacking onto p3 for the subsequent objects grabbed
    
    # Height and drop location
    # Case no object is present at the first drop point p1, drop to p1 area
    if(len(pose) == 0 or list_len == 0):
        height = 50.0
        d_drop0_blue = np.array([120., 520., height, 1]).reshape((4,1))
        d_drop0_red = np.array([-120., -520., height, 1]).reshape((4,1))

    # Case already 3 objects at p1, switch to the next drop point p2
    elif(list_len == 3):
        height = 50.0
        d_drop0_blue = np.array([70., 520., height, 1]).reshape((4,1))
        d_drop0_red = np.array([-70., -520., height, 1]).reshape((4,1))

    # Case already 2 objects at p2 (a total of 5 objects on the platform), drop to the 3rd location p3
    elif(list_len == 5):
        height = 50.0
        d_drop0_blue = np.array([120., 470., height, 1]).reshape((4,1))
        d_drop0_red = np.array([-120., -470., height, 1]).reshape((4,1))

    # else drop on top of the previously dropped object (the top object on the stack at drop point p_i)
    else:
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

        # Angle btw X_base axis and drop location (vector)
        d_drop1_blue_xy = np.array([d_drop1_blue[0], d_drop1_blue[1], 0])
        cos_theta = ((np.array([1,0,0]).dot(d_drop1_blue_xy)) / np.linalg.norm(d_drop1_blue_xy))[0]
        theta = np.arccos(cos_theta)

        # Update Height and calculate x,y offset based on relative position of the grabbed object wrt the gripper
        if (len(object_pose)):
            obj_0 = np.matmul(T01, np.array(object_pose))
            Z_obj = obj_0[2,-1]
            X_obj = obj_0[0,-1]
            Y_obj = obj_0[1,-1]

            positions, _ = FK.forward(qstart)
            Z_e = positions[-1,2]
            height += (Z_e - Z_obj)
            X_offset = positions[-1,0] - X_obj
            Y_offset = positions[-1,1] - Y_obj

        # Te1: drop off pose ; apply the offset to the desired gripper position for dropping the object at drop point p_i
        Te1 = np.array([[np.sin(theta), 0, cos_theta, d_drop1_blue[0,0] + X_offset],
                        [cos_theta, 0, -np.sin(theta), d_drop1_blue[1,0] - 3],
                        [0, 1, 0, height],
                        [0, 0, 0, 1]])

    # same implemenation as the 'blue'
    elif(str(color).lower() == 'red'):

        T01 = np.array([[1, 0, 0, 200],
                       [0, 1, 0, 200],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        d_drop1_red = np.matmul(T01, d_drop0_red) #drop location in base frame

        d_drop1_red = d_drop1_red[0:3,:]

        # Angle btw X_base axis and drop location
        d_drop1_red_xy = np.array([d_drop1_red[0], d_drop1_red[1], 0])
        cos_theta = ((np.array([1,0,0]).dot(d_drop1_red_xy)) / np.linalg.norm(d_drop1_red_xy))[0]
        theta = np.arccos(cos_theta)

        # Update Height
        if (len(object_pose)):
            obj_0 = np.matmul(T01, np.array(object_pose))
            Z_obj = obj_0[2,-1]
            X_obj = obj_0[0,-1]
            Y_obj = obj_0[1,-1]

            positions, _ = FK.forward(qstart)
            Z_e = positions[-1,2]
            height += (Z_e - Z_obj)
            X_offset = positions[-1,0] - X_obj
            Y_offset = positions[-1,1] - Y_obj

        Te1 = np.array([[np.sin(theta), 0, cos_theta, d_drop1_red[0,0] + X_offset],
                        [cos_theta, 0, -np.sin(theta), d_drop1_red[1,0] - 3],
                        [0, 1, 0, height],
                        [0, 0, 0, 1]])

    else:
        print("incorrect color input")
        return


    #############################################
    # Compute IK for Drop off pose
    #############################################

    print("height")
    print(height)
    print("offsets")
    print(X_offset, Y_offset)

    IK = calculateIK()

    q_drop, isPos = IK.inverse(Te1)


    #############################################
    # Plan Path to Drop off pose
    #############################################

    path = []

    if(not isPos):
        print('Not feasible pose')
        return path

    # Define intermediate point
    Tinter1 = deepcopy(Te1)

    Tinter1[0:3,-1] = np.array([Te1[0,-1] - 30, Te1[1,-1] + 70, height + 70])
    
    q_inter, isPos = IK.inverse(Tinter1)

    # Add intermediate pose
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
        dq2[-1] += release_count
        q_release = q_release + dq2
        path.append(q_release)
        release_count += 1

    for i in range(10):
        q_release[1] -= 0.03
        q_release[2] -= 0.01

        path.append(q_release)

    return path
    


##########################################################################
# Control Lynx to drop

def drop_object(lynx, color, object_index, value):
    """
    This function calls drop and use Lynx.ArmController to execute the drop path
    :param lynx:            ArmController instance
    :param color:           string of color of the robot we are using (blue or red)
    :param object_index:    Index of the object currently dropping (get from pickStatic)
    :param value:           value returned by pickDynamic()
                            (1 if dynamic object, 2 if static object)

    :return:
        None
    """

    qStart, vel = lynx.get_state()
    [name, pose, twist] = lynx.get_object_state()

    # get pose of the grabbed object to be dropped
    if(object_index == -1):
        object_pose = []
    else:
        object_pose = pose[object_index]

    # get pose of all cubes on green plate
    new_list = []
    if(color == "blue"):
        #list: information of all objects on the green platform 
        list = [[pose[i], pose[i][2,3]] for i in range(len(name)) if (pose[i][1,3]>0 and pose[i][1,3] > 440 and pose[i][2,3]>0) ]
        
        # new_list: pose and height of the object(s) stacked at p2 
        if(len(list) >= 3 and len(list) < 5):
            new_list = [[pose[i], pose[i][2,3]] for i in range(len(name)) if (pose[i][1,3]>0 and pose[i][1,3] > 440 and pose[i][2,3]>0 and pose[i][0,3] < 95) ]
        # new_list: pose and height of the object(s) stacked at p3
        elif(len(list) >= 5):
            new_list = [[pose[i], pose[i][2,3]] for i in range(len(name)) if (pose[i][1,3]>0 and pose[i][1,3] > 440 and pose[i][2,3]>0 and pose[i][1,3] < 490) ]

    else:
        list = [ [pose[i], pose[i][2,3]] for i in range(len(name)) if (pose[i][1,3]<0 and pose[i][1,3] < -440 and pose[i][2,3]>0)]
        if(len(list) >= 3 and len(list) < 5):
            new_list = [[pose[i], pose[i][2,3]] for i in range(len(name)) if (pose[i][1,3]<0 and pose[i][1,3] < -440 and pose[i][2,3]>0 and pose[i][0,3] > -95) ]
        elif(len(list) >= 5):
            new_list = [[pose[i], pose[i][2,3]] for i in range(len(name)) if (pose[i][1,3]<0 and pose[i][1,3] < -440 and pose[i][2,3]>0 and pose[i][1,3] > -490) ]


    # case dropping the first object
    if(len(list) == 0):
        path = drop(qStart, color, [], 0, object_pose)

    else:
        # sort the list and new list based on heights of objects stored in them
        list = sorted(list, key=operator.itemgetter(1))
        new_list = sorted(new_list, key=operator.itemgetter(1))

        # set the initial target location based on the top object at p_i
        if(len(list) < 3 or len(new_list) == 0):
            target_pose = list[-1][0]

            target_loc = target_pose[0:3,-1]
        else:
            target_pose = new_list[-1][0]

            target_loc = target_pose[0:3,-1]

        list_len = len(list)

        path = drop(qStart, color, target_loc, list_len, object_pose)

    # iterate over target waypoints
    loop_iter = 0
    for p in path:
        print("Goal:")
        print(p)

        # value = 1 if pick dynamic
        # value = 2 if pick static
        if(loop_iter == 0 and value == 2):
            lynx.set_pos(np.ravel(p))
        elif(loop_iter == 1):
            lynx.set_pos(np.ravel(p))
        else:
            lynx.command(np.ravel(p))

        while True:
            sleep(0.005)
            pos, vel = lynx.get_state()

            if np.linalg.norm(np.array(pos[0:4])- np.array(p[0:4])) <= 0.05:
                break

        loop_iter+= 1