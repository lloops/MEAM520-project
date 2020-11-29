#!/usr/bin/python2
from time import sleep
import numpy as np
from calculateFK import calculateFK
from IKv import IK_velocity
from copy import deepcopy
from calculateIK import calculateIK
import operator

class pickStatic:

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
        slp_time = 0.2
        path_counter = 0
        for p in path:
            print("Goal:")
            print(p)

            self.lynx.set_pos(np.ravel(p))
            # sleep(slp_time)
            # slp_time = slp_time / 1.005
            if(path_counter > 1):
                sleep(0.15)
            elif(path_counter == 0):
                sleep(0.5)
            else:
                sleep(2.3)

            path_counter += 1


    def pick(self):
        [name, pose, twist] = self.lynx.get_object_state()

        #1. if the object's name is static;
        #2. FOR BLUE: if the y coord of the obj is > 0 (-y coord are opponent's)
        #3. sometimes when simulation is initalized., a block may get struck off from the platform, which would fall to z coord (world frame) ~= -999
        if(self.color == "blue"):
            static_lst = [ [i] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]>0 and pose[i][1,3] < 440 and pose[i][2,3]>0) ]
        else:
            static_lst = [ [i] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]<0 and pose[i][1,3] > -440 and pose[i][2,3]>0) ]

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
        dobj1 = Tobj1[0:3,-1] + np.array([5, -5, -8])

        # reach point
        dreach1 = dobj1 + np.array([-50, 25, 40])

        # initial v and w
        Vreach1 = dreach1 - de1
        Wreach1 = np.array([0,0,0])

        #q = deepcopy(self.qstart)
        dt = 0.1

        path = []

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

        q_reach_IK = np.append(np.ravel(q_reach_IK)[range(4)], [-1.57,30])

        q = deepcopy(self.qstart)

        path = []

        path.append(np.ravel(q))

        path.append(q_reach_IK)

        #####################################################################
        # Velocity IK Method

        # # Move Gripper to reach point
        # for i in range(25):
        #     dq = IK_velocity(q, Vreach1, Wreach1, 6)
        #     q = q + dq * dt
        #
        #     qcopy = np.ravel(q)
        #     if(any([(qcopy[j] > self.upperLim[j] or qcopy[j] < self.lowerLim[j]) for j in range(6)])):
        #         break
        #
        #     jpos, T0e = FK.forward(q)
        #     de1 = jpos[-1,:]
        #     Vreach1 = dreach1 - de1
        #
        #     path.append(q)

        ######################################################
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


        # Move Gripper to Object
        q_reach = np.ravel(path[-1])
        jpos, T0e = FK.forward(q_reach)
        de1 = jpos[-1,:]
        V_grab = dobj1 - de1

        for i in range(20):
            dq = IK_velocity(q_reach, V_grab, Wreach1, 6)
            q_reach = q_reach + dq * dt

            jpos, T0e = FK.forward(q_reach)
            de1 = jpos[-1,:]
            V_grab = dobj1 - de1

            path.append(q_reach)

        #close gripper
        q_reach =  np.ravel(path[-1])
        for i in range(5):
            dq2 = [0,0,0, 0, 0, -6]
            q_reach = q_reach + dq2
            path.append(q_reach)

        # Move back
        q_reach =  np.ravel(path[-1])
        for i in range(5):
            dq2 = [0,-0.1, 0, 0, 0, 0]
            q_reach = q_reach + dq2
            path.append(q_reach)

        self.move(path)
