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
        

    # Control lynx to execute the path
    def move(self, path):
        #slp_time = 0.2
        path_counter = 0
        
        #execute the path
        for p in path:
            print("Goal:")
            print(p)

            self.lynx.command(np.ravel(p))

            # 3. for all subsequent waypoints, sleeps 0.1s 
            if(path_counter > 1):
                sleep(0.1)
            
            # 1.sleeps for 0.5s to wait at the qStart([0, 0, 0, 0, 0, 0]) to ensure smoother and less abrupt transition 
            # from last executed function (e.g.drop/pick dynamic) into pickStatic
            elif(path_counter == 0):
                sleep(0.5)
                
            # 2.sets gripper at reach point (q_reach_IK); needs relatively longer time 
            # since uses only one configuration, sleeps 0.5s
            else:
                sleep(0.5)

            path_counter += 1


    # pick() plan and execute a path
    # Return the index of target currently picking
    def pick(self):
        [name, pose, twist] = self.lynx.get_object_state()

        ###########################################################
        # Obtain the Current Closest Static Object (euclidean distance measured from base frame
        # origin to the position of a static object (in base frame coordinates))
        ###########################################################
        
        #1. if the object's name is static;
        #2. FOR BLUE: if the y coord of the obj is > 0 (-y coord are opponent's); vice versa for RED;
        #3. ignore those objects on the green platform (drop off platform) ( pose[i][1,3] < 440 (blue) or > -440 (red) );
        #4. ignore those block that fell off from the platform, which would have z coordinate (world frame) ~= -999 (pose[i][2,3]>0);
        
        if(self.color == "blue"):
            static_lst = [ [i] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]>0 and pose[i][1,3] < 440 and pose[i][2,3]>0) ]
        else:
            static_lst = [ [i] for i in range(len(name)) if (name[i][5]=="s" and pose[i][1,3]<0 and pose[i][1,3] > -440 and pose[i][2,3]>0) ]

        #return -1 to indicate no more static objects to pick
        if (len(static_lst) == 0):
            return -1

        
        # sort the static objects from closest to furthest, 
        # set the closest object as target.
        for j in range(len(static_lst)):
            Tobj_1 = self.T01.dot(pose[static_lst[j][0]])
            dist_1 = np.linalg.norm(Tobj_1[0:3,3])
            static_lst[j].append(dist_1)

        static_lst = sorted(static_lst, key=operator.itemgetter(1))
        target = static_lst[0][0]
        #


        ###########################################################
        # Transform and Initialize for FK, IK, and Velocity IK
        ###########################################################
        # NOTE: frame 1 denotes the base frame of the robot
 
        # get transformation matrix from object (obj) to base frame (1)
        Tobj0 = np.array(pose[target]) #target object in world frame
        Tobj1 = np.matmul(self.T01, Tobj0) #target object in base frame

        FK = calculateFK()
        
        #get joint positions
        jpos, _ = FK.forward(self.qstart)

        # end effector position
        de1 = jpos[-1,:]

        # target position (adjusted based on Tobj1, the object position)
        dobj1 = Tobj1[0:3,-1] + np.array([5, -5, -8])

        # set reach point
        dreach1 = dobj1 + np.array([-50, 25, 60])


        ###########################################################
        # Stage 1: IK Method
        ###########################################################
        # calculate angle theta between +X axis of base frame and the dreach1 
        # (vector from base frame origin to reach point)
        dreach_xy = np.array([dreach1[0], dreach1[1], 0])
        cos_theta = ((np.array([1,0,0]).dot(dreach_xy)) / np.linalg.norm(dreach_xy))
        theta = np.arccos(cos_theta)

        # transformation matrix that specifies the desired end effector orientation at the reach point
        Te1 = np.array([[np.sin(theta), 0, cos_theta, dreach1[0]],
                        [cos_theta, 0, -np.sin(theta), dreach1[1]],
                        [0, 1, 0, dreach1[2]],
                        [0, 0, 0, 1]])

        IK = calculateIK()

        # calculate the configuration corresponding to Te1
        q_reach_IK, isPos = IK.inverse(Te1)
        # adjust gripper to be fully open with its opening parallel to ground
        q_reach_IK = np.append(np.ravel(q_reach_IK)[range(4)], [-1.57,30])
        q = deepcopy(self.qstart)

        # stores all configurations needed to perform the pickStatic function 
        path = []

        # 1st waypoint: qStart = [0, 0, 0, 0, 0, 0]
        path.append(np.ravel(q))
        # 2nd point: to the reach point
        path.append(q_reach_IK)

        ######################################################
        # Path to move the griper to grab target
        ####################################################
        q_reach = np.ravel(path[-1])

        # in case the execution of q_reach_IK doesn't fully open the gripper and set it parallel to ground
        # use more waypoints to open and rotate the gripper 
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

        ######################################################
        # Stage 2: Velocity IK to Approach Target
        ####################################################
        
        # Stage 2 starts when gripper has already move to the reach point
        q_reach = np.ravel(path[-1])
        jpos, _ = FK.forward(q_reach)
        de1 = jpos[-1,:]
        
        # the initial desired linear velocity of the gripper (to approach the object)
        V_grab = dobj1 - de1
        # angular velocity of end effector always set to 0 to maintain preset orientation 
        W_grab = np.array([0,0,0])
  
        #small increment in time
        dt = 0.1    
        
        # iterate over 30 loops to get 30 waypoints for approaching the target; 
        # when at the end of 30 iterations, ideally the gripper should be near the object. 
        for i in range(30): 
            
            # get joint change; 6: end effector
            dq = IK_velocity(q_reach, V_grab, W_grab, 6)
            # update the next configuration
            q_reach = q_reach + dq * dt

            # update de1 based on the new q (not real-time update, de1 is the ideal value)
            jpos, _ = FK.forward(q_reach)
            de1 = jpos[-1,:]
            
            # update the desired lin. vel. of end effector
            V_grab = dobj1 - de1

            # add the calculated q to path
            path.append(q_reach)


        # Now after 30 waypoints, the gripper arrives at the target object 
        # close the gripper to grab the object
        q_reach =  np.ravel(path[-1])
        
        for i in range(5):
            dq2 = [0,0,0, 0, 0, -6]
            q_reach = q_reach + dq2
            path.append(q_reach)
        #


        # Safely move away from the platform by lifting the arm
        q_reach =  np.ravel(path[-1])
        for i in range(5):
            dq2 = [0,-0.1, 0, 0, 0, 0]
            q_reach = q_reach + dq2
            path.append(q_reach)
        #


        # The path is complete; calls move() to execute the path
        self.move(path)

        # return the target index
        return target
    
    