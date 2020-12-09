import numpy as np
import math

class calculateFK():

    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable

        """
        # Lynx Dimensions in mm
        self.L1 = 76.2    # distance between joint 0 and joint 1
        self.L2 = 146.05  # distance between joint 1 and joint 2
        self.L3 = 187.325 # distance between joint 2 and joint 3
        self.L4 = 34      # distance between joint 3 and joint 4
        self.L5 = 34      # distance between joint 4 and center of gripper

        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    def forward(self, q):
        """
        INPUT:
        q - 1x6 vector of joint inputs [q0,q1,q2,q3,q4,lg]

        OUTPUTS:
        jointPositions - 6 x 3 matrix, where each row represents one
                  joint along the robot. Each row contains the [x,y,z]
                  coordinates of the respective joint's center (mm). For
                  consistency, the first joint should be located at
                  [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  base (0) frame
        """
        # Your code starts here

        jointPositions = np.zeros((6,3))
        T0e = np.identity(4)

        ###---------------------###
        ## establish DH parameters
        a_lst = [0, self.L2, self.L3, 0, 0]
        alp_lst = [-math.pi/2, 0, 0, -math.pi/2, 0]

        d_lst = [self.L1, 0, 0, 0, self.L4 + self.L5]

        q = np.array(q).reshape(6)

        theta_lst = [q[0], q[1] - math.pi/2, q[2] + math.pi/2, q[3] - math.pi/2, q[4]]

        ###---------------------###
        #position of center of joint 5 wrt frame 4
        p_4 = np.array(
                [0, 0, 34, 1]
                )


        for i in range(5):

            T0e = T0e.dot( self.getA( a_lst[i], alp_lst[i], d_lst[i], theta_lst[i] ) )

            pos = T0e[:-1, -1]

            #to calculate position of (the center of) joint 5,
            #since o4 is not located at the center of joint 5,
            #transform p_4 (position of center of joint 5 wrt frame 4) into ... wrt frame 0
            if i == 3:
                pos = T0e.dot(p_4).reshape(4,1)[:-1, -1]

            #joint 1's default position is 0,0,0; start recording joint i's position from the 2nd joint (i+1 index)
            jointPositions[i+1] = pos

        # Your code ends here
        return jointPositions, T0e



    def getA(self, a, alp, d, theta):

        sin_theta, cos_theta = math.sin(theta), math.cos(theta)
        sin_alp, cos_alp = math.sin(alp), math.cos(alp)

        A = np.array([
                         [cos_theta, -sin_theta * cos_alp, sin_theta * sin_alp, a*cos_theta], #row1
                         [sin_theta, cos_theta * cos_alp, -cos_theta * sin_alp, a*sin_theta],  #row2
                         [0, sin_alp, cos_alp, d],
                         [0, 0, 0, 1]

                         ])

        return A

        
