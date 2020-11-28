#!/usr/bin/python2
import numpy as np
import math
from numpy import pi

class calculateIK():

    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable

        """
        # Lynx ADL5 constants in mm
        self.d1 = 76.2                      # Distance between joint 1 and joint 2
        self.a2 = 146.05                    # Distance between joint 2 and joint 3
        self.a3 = 187.325                   # Distance between joint 3 and joint 4
        self.d4 = 34.0                        # Distance between joint 4 and joint 5
        self.d5 = 68.0                        # Distance between joint 4 and end effector

        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)


    def coerce(self,theta):
        """
        input:
        theta: floating point, an angle in radians value to be coerced

        output:
        newTheta: floating point, an angle in radians between -pi and pi
        """
        newTheta = theta
        # determine if it's between -360 and 360 degrees:
        if (newTheta > 2*pi):
            while (newTheta > 2*pi):
                newTheta = newTheta - 2*pi
        elif (newTheta < -2*pi):
            while (newTheta < -2*pi):
                newTheta = newTheta + 2*pi

        # coerce to be between -pi and pi
        if (newTheta <-pi):
            newTheta = newTheta + 2*pi
        elif (newTheta > pi):
            newTheta = newTheta - 2*pi

        return newTheta


    def computeIK(self, T0e):
        translation = T0e[:, -1]
        x0e = translation[0]
        y0e = translation[1]
        z0e = translation[2]

        #Compute Wrist Center
        xc = x0e - self.d5 * T0e[0,2]
        yc = y0e - self.d5 * T0e[1,2]
        zc = z0e - self.d5 * T0e[2,2]

        #Compute Joint 1 (Only 1 possible solution in range [-pi, pi])
        T = T0e.copy()
        theta1 = np.arctan2(yc,xc)

        # coerce theta1 to be in the range [-pi, pi]
        if (theta1 > pi/2):
             theta1 = theta1 - pi
        elif (theta1 < -pi/2):
             theta1 = theta1 + pi


        #Compute Joint 3 (2 solutions for elbow up and down)
        theta3Down = -pi/2 - np.arccos((xc**2 + yc**2 + (zc - self.d1)**2 - self.a2**2 - self.a3**2)/(2*self.a2*self.a3))
        theta3Down = 2*pi + theta3Down
        theta3Up = pi - theta3Down

        #Compute Joint 2 (2 solutions for each theta3)
        theta2Down =  pi/2 - np.arctan2((zc-self.d1),(xc*1/np.cos(theta1))) - np.arctan2((self.a3*np.cos(theta3Down)),(self.a2 - self.a3*np.sin(theta3Down)))
        theta2Up = pi/2 - np.arctan2((zc-self.d1),(xc*1/np.cos(theta1))) - np.arctan2((self.a3*np.cos(theta3Up)),(self.a2 - self.a3*np.sin(theta3Up)))


        #Compute joint 4 (2 solutions for each pair of theta 2, 3)
        # calculate theta 4 twice, because it could also be a negative angle
        sinTheta4Down = -np.cos(theta2Down+theta3Down)*T[2,2] - np.cos(theta1)*T[0,2]*np.sin(theta2Down+theta3Down)\
            - T[1,2]*np.sin(theta1)*np.sin(theta2Down+theta3Down)
        cosTheta4Down = -np.sin(theta2Down+theta3Down)*T[2,2] + np.cos(theta1)*T[0,2]*np.cos(theta2Down+theta3Down)\
            + T[1,2]*np.sin(theta1)*np.cos(theta2Down+theta3Down)
        theta4Down    = np.arctan2(sinTheta4Down,cosTheta4Down) # covers if theta4 is above 90 degrees


        sinTheta4Up = -np.cos(theta2Up+theta3Up)*T[2,2]     - np.cos(theta1)*T[0,2]*np.sin(theta2Up+theta3Up) \
            - T[1,2]*np.sin(theta1)*np.sin(theta2Up+theta3Up)
        cosTheta4Up = -np.sin(theta2Up+theta3Up)*T[2,2] + np.cos(theta1)*T[0,2]*np.cos(theta2Up+theta3Up)\
            + T[1,2]*np.sin(theta1)*np.cos(theta2Up+theta3Up)
        theta4Up    = np.arctan2(sinTheta4Up,cosTheta4Up) #in case it's below -90


        #Compute joint 5 (1 solution depending on theta1)
        sinTheta5 = T[0,0]*np.sin(theta1) - np.cos(theta1)*T[1,0]
        cosTheta5 = T[0,1]*np.sin(theta1) - np.cos(theta1)*T[1,1]
        theta5 = np.arctan2(sinTheta5,cosTheta5)


        ######################################
        # define q

        q = np.zeros([2, 5])

        # convenience variables
        theta1Up = theta1
        theta1Down = theta1
        # create the list of vectors
        q[0,:] = [theta1Up, theta2Up,theta3Up,theta4Up,theta5]
        q[1,:] = [theta1Down,theta2Down,theta3Down,theta4Down,theta5]

        # coerce all the variables to be between -pi and pi
        for i in range(q.shape[0]):
            for j in range(q.shape[1]):
                q[i,j] = self.coerce(q[i,j])

        return q

    def inverse(self, T0e):
        """
        INPUT:
        T - 4 x 4 homogeneous transformation matrix, representing
           the end effector frame expressed in the base (0) frame
           (position in mm)

        OUTPUT:
        q - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad)
           which are required for the Lynx robot to reach the given
           transformation matrix T. Each row represents a single
           solution to the IK problem. If the transform is
           infeasible, q should be all zeros.
        isPos - a boolean set to true if the provided
             transformation T is achievable by the Lynx robot as given,
             ignoring joint limits
        """
        isPos = 1

        #########################################
        #check outside workspace
        translation = T0e[:, -1]
        x0e = translation[0]
        y0e = translation[1]
        z0e = translation[2]

        xy_distance = np.sqrt(x0e**2 + y0e**2)
        z_distance = np.sqrt(xy_distance**2 + z0e**2)

        #range of reachable distance in xy plane
        xyrange = self.a2 + self.a3 + self.d5

        #range of reachable distance in z
        zrange_pos = self.d1 + xyrange
        zrange_neg = self.d1 - xyrange

        #check if d0e is in the reachable range
        if(xy_distance > xyrange):
            print("outside xyrange")
            isPos = 2
            return np.ndarray((1,5)), isPos

        elif(z0e > 0 and z_distance > zrange_pos):
            print("outside zrange_pos")
            isPos = 2
            return np.ndarray((1,5)), isPos

        elif(z0e < 0 and z_distance > zrange_neg):
            print("outside zrange_neg")
            isPos = 2
            return np.ndarray((1,5)), isPos


        ##########################################
        #check feasibility and closest transformation
        xc = x0e - self.d5 * T0e[0,2]
        yc = y0e - self.d5 * T0e[1,2]
        zc = z0e - self.d5 * T0e[2,2]

        #Define the plane formed by wrist center position and Z0 axis
        cross_prod = np.cross([xc, yc, zc], [0,0,1])
        ze_axis = T0e[0:3, 2]

        if(abs(np.dot(ze_axis, cross_prod)) > 0.05):
            print('infeasible')

            #projection of Ze onto plane
            norm_ = np.sqrt(np.sum(np.square(cross_prod)))
            proj_z = ze_axis - (np.dot(ze_axis, cross_prod)/norm_**2)*cross_prod

            #convert to unit vector
            proj_z = proj_z / np.sqrt(np.sum(np.square(proj_z)))

            #We cross projected Ze with Z0 to find a feasible Ye
            proj_y = np.cross(proj_z, np.array([0,0,1]))
            proj_y = proj_y / np.sqrt(np.sum(np.square(proj_y)))

            #Cross Ye, Ze to get new Xe
            proj_x = np.cross(proj_y, proj_z)


            #compute transformation matrix to rotate frame e about Xe by angle theta
            Tnew = np.array([[proj_x[0], proj_y[0], proj_z[0], x0e], \
                            [proj_x[1], proj_y[1], proj_z[1], y0e],\
                            [proj_x[2], proj_y[2], proj_z[2], z0e],\
                            [0,0,0,1]])
            # print("T0e:")
            # print(T0e)
            # print("Transform to Closest Te0:")
            # print(Tnew)

            #Re-Compute IK for new T0e
            q = self.computeIK(Tnew)
            #q = self.computeIK(T0e)
            isPos = 1
        else:
            q = self.computeIK(T0e)

        #q = self.computeIK(T0e)

        ##########################################
        #check joint limit
        qcopy = q.copy()
        for i in range(qcopy.shape[0]-1,-1,-1):
            thetas = qcopy[i]
            if ((thetas > self.upperLim[0,0:5]).any()):
                print("Deleting solution out of upper limit")
                q = np.delete(q,i,axis=0)

            elif ((thetas < self.lowerLim[0,0:5]).any()):
                print("Deleting solution out of lower limit")
                q = np.delete(q,i,axis=0)

            elif ((abs(x0e)<10**(-6)) and (abs(y0e)<10**(-6))):
                q[i,4]= float('nan')
                q[i,0]= float('nan')

        #############################################
        #Outputting remaining solutions
        # remove non-unique rows, but only if q isn't empty to avoid errors
        if (len(q) > 0):
            # the very special case if it's straight up at edge of reachable workspace
            # this is needed because the elbow up and elbow down solutions for
            # this are unique enough not to get removed by unique()
            if ((np.isnan(q).any()) and (q[0,2]==-pi/2)):
                q = np.array([q[0,:]])
            else:
                q = np.unique(q,axis=0)
        else:
            q = np.ndarray((1,5))
            isPos = 0

        #print("Joint Variables")
        np.set_printoptions(precision=3,suppress=True)
        #print(q)
        return q, isPos





# if __name__=='__main__':
#     main = Main()
#     #Zero Position
#     T0e = np.array([[0,0,1,255.325],[0,-1,0,0],[1,0,0,222.25],[0,0,0,1]])

#     d1 = 76.2                      # Distance between joint 1 and joint 2
#     a2 = 146.05                    # Distance between joint 2 and joint 3
#     a3 = 187.325                   # Distance between joint 3 and joint 4
#     d4 = 34                        # Distance between joint 4 and joint 5
#     d5 = 68

#     #Upperlim
#     #T0e = np.array([[0.982,0,0.186,-308.894],[0,-1,0,0.002],[0.186,0,-0.982,88.755],[0,0,0,1]])

#     #zero position with not feasible orientation
#     #T0e = np.array([[0,0.1,0.9,255.325], [0,-0.9,0.1,0],[1,0,0,222.25],[0,0,0,1]])

#     # T0e = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,d1+a2+a3+d5-150],[0,0,0,1]]) # straight up at limits

#     # Target 1
#     # T0e = np.array([[   0.019,    0.969,    0.245,   47.046],[   0.917,   -0.115,    0.382,   73.269],[   0.398 ,   0.217,   -0.891,  100.547],[   0.,       0. ,      0.,       1.]])

#     # Target 2
#     # T0e = np.array([[  -0.993,   -0.,       0.119,  -96.936],[   0.,      -1.,      -0.,       0.   ],[   0.119,    0.,       0.993,  401.229],[   0. ,      0.  ,     0.  ,     1.   ]])

#     # Target 3
#     # T0e = np.array([ [-0.3409003, -0.1074855,  0.9339346, 282.96],[0.7842780, -0.5802868,  0.2194888, -48.302],[0.5183581,  0.8072881,  0.2821184, 235.071 ], [0,0,0,1]])

#     # Target 4
#     # T0e = np.array([[  0.5054096, -0.8370580, -0.2095115, -45],[-0.0305796,  0.2252773, -0.9738147,-300],[0.8623375,  0.4985821,  0.0882604, 63 ],[0,0,0,1]])


#     (q, isPos) = main.inverse(T0e)
#     #q = main.computeIK(T0e)
#     np.set_printoptions(precision=3,suppress=True)
#     #print(q)
