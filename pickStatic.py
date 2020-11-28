#!/usr/bin/python2
from time import sleep
import numpy as np
from calculateFK import calculateFK
from IKv import IK_velocity
from copy import deepcopy
import operator

class pickStatic:

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
    def __init__(self, lynx, color):

        if (str(color).lower() == 'blue'):
            self.T01 = np.array([[-1, 0, 0, 200],
                        [0, -1, 0, 200],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        elif(str(color).lower() == 'red'):
            self.T01 = np.array([[1, 0, 0, 200],
                        [0, 1, 0, 200],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        else:
            print("incorrect color input")
            return

        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15])
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30])

        self.color = color
        self.lynx = lynx
        self.qstart = [0, 0, 0, 0, 0, 0]


    def move(self, path):
        slp_time = 0.5
        for p in path:
            print("Goal:")
            print(p)

            self.lynx.set_pos(np.ravel(p))
            sleep(slp_time)
            slp_time = slp_time / 1.005


    def pick(self):
        [name, pose, twist] = self.lynx.get_object_state()

        #1. if the object's name is static;
        #2. FOR BLUE: if the y coord of the obj is > 0 (-y coord are opponent's)
        #3. sometimes when simulation is initalized., a block may get struck off from the platform, which would fall to z coord (world frame) ~= -999
        if(self.color == "blue"):
            static_lst = [ [i] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]>0 and pose[i][2,3]>0) ]   
        else:
            static_lst = [ [i] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]<0 and pose[i][2,3]>0) ]
        
        for j in range(len(static_lst)):
            Tobj_1 = self.T01.dot(pose[static_lst[j][0]])
            dist_1 = np.linalg.norm(Tobj_1[0:3,3])
            static_lst[j].append(dist_1)

        static_lst = sorted(static_lst, key=operator.itemgetter(1))
        target = static_lst[0][0]

        # get transformation matrix from object to base frame

        Tobj0 = np.array(pose[target]) #target object in world frame
        Tobj1 = np.matmul(self.T01, Tobj0) #target object in base frame

        FK = calculateFK()

        jpos, T0e = FK.forward(self.qstart)

        # end effector position
        de1 = jpos[-1,:]

        # target position
        dobj1 = Tobj1[0:3,-1] + np.array([5, -5, -5])

        # reach point
        dreach1 = dobj1 + np.array([-70, 30, 30])

        # initial v and w
        Vreach1 = dreach1 - de1
        Wreach1 = np.array([0,0,0])

        q = deepcopy(self.qstart)
        dt = 0.1

        path = []

        # Move Gripper to reach point
        for i in range(25):   
            dq = IK_velocity(q, Vreach1, Wreach1, 6)
            q = q + dq * dt

            qcopy = np.ravel(q)
            if(any([(qcopy[j] > self.upperLim[j] or qcopy[j] < self.lowerLim[j]) for j in range(6)])):
                break

            jpos, T0e = FK.forward(q)
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
        jpos, T0e = FK.forward(q_reach)
        de1 = jpos[-1,:]
        V_grab = dobj1 - de1

        for i in range(20):
            dq = IK_velocity(q, V_grab, Wreach1, 6)
            q_reach = q_reach + dq * dt

            jpos, T0e = FK.forward(q_reach)
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

        self.move(path)