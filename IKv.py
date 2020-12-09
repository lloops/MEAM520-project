import numpy as np
import math


#note: all input/output are in np.array (1,n) shape

def IK_velocity (q, v, omega, joint):
    """
    :param q: 1 x 6 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any element is Nan, then that velocity can be
                  anything
    :param joint: an integer in [0,6] corresponding to which joint you are tracking
    :return:
    dq - 1 x 6 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error.

    """
    d1 = 76.2                      # Distance between joint 0 and joint 1
    a2 = 146.05                    # Distance between joint 1 and joint 2
    a3 = 187.325                   # Distance between joint 2 and joint 3
    d4 = 34                        # Distance between joint 3 and joint 4
    d5 = 68                        # Distance between joint 3 and joint 5
    lg = 0                         # Distance between joint 5 and end effector (gripper length)

    dq = np.array([0, 0, 0, 0, 0, 0])

    #get jacobian
    j_mat = Jacobian(q, joint)

    v = v.reshape(1,3)
    omega = omega.reshape(1,3)
    #stack v, omega together
    body_v = np.hstack((v, omega))

    #calls checkUnconstr that returns a list on indices of the unconstrained v (nan) in body_v
    unconstr_inds = checkUnconstr(body_v)
    #remove uncontrained nan from body_v (delete col-ind)
    body_v = np.delete(body_v, unconstr_inds, axis = 1)
    #remove corresponding rows from J (delete row-ind)
    j_mat = np.delete(j_mat, unconstr_inds, axis = 0)

    #calculate dq, multiply j_+ (pseudoinv obtained from SVD) and body_v
    dq = np.linalg.pinv(j_mat).dot(body_v.transpose())
    #reshape into (1,6)
    dq = dq.transpose()

    return dq


##get jacobian for the joint being tracked, at config = q
def Jacobian(q, joint):

    #init J as a 6*5 zero-matrix
    j_mat = np.zeros((6,6))

    #if asking for the body_v of joint 0 (link0/base) or joint 1, then return J as a zero-matrix
    if (joint == 0 or joint == 1):
        return j_mat

    T0e = np.identity(4)

    ###---------------------###
    L1 = 76.2    # distance between joint 0 and joint 1
    L2 = 146.05  # distance between joint 1 and joint 2
    L3 = 187.325 # distance between joint 2 and joint 3
    L4 = 34      # distance between joint 3 and joint 4
    L5 = 34      # distance between joint 4 and center of gripper

    ## establish DH parameters (frame1,2,3,4,5)
    a_lst = [0, L2, L3, 0, 0]
    alp_lst = [-math.pi/2, 0, 0, -math.pi/2, 0]
    d_lst = [L1, 0, 0, 0, L4 + L5]


    q = np.array(q)
    q = q.reshape(6)
    theta_lst = [q[0], q[1] - math.pi/2, q[2] + math.pi/2, q[3] - math.pi/2, q[4]]

    ###---------------------###

    #position of center of joint 5 wrt frame 4
    p_4 = np.array(
            [0, 0, 34, 1]
                )


    #init. with z_0_0, and o_0_0 (the position of origin of fr0, and the joint 1)
    z_i_0_lst = [np.array([0, 0, 1])]
    o_i_0_lst = [np.array([0, 0, 0])]

    #calculate o_1 ... o_q and z_1 ... z_q; joint = 2...6
    for i in range(joint-1):

        T0e = T0e.dot( getA( a_lst[i], alp_lst[i], d_lst[i], theta_lst[i] ) )

        #record z_i_0
        z_i_0_lst.append(T0e[0:3, 2])
        #record o_i_0 (the origin of i-th DH frame wrt to frame0)
        o_i_0_lst.append(T0e[:-1, -1])


        #pos is the i-th joint pos
        pos = T0e[:-1, -1]
        #the only joint pos doesn't coincide with origin of DH frame is position of joint 4
        if i == 3:
            pos = T0e.dot(p_4).reshape(4,1)[:-1, -1]


    #record o_n as the position of joint q wrt frame0
    o_n = pos


    #last elements of z_i_0_lst and o_i_0_lst are useless, since we only need up to z_(i-1) and o_(i-1) for calculation of J for joint i
    #we can still keep them since the loop to calculate and fill J won't access the last elems
#    z_i_0_lst.pop()
#    o_i_0_lst.pop()


    #min possible joint value is 2 (joint 0,1 has J = 0s); i: i-th col of J (i=1,2,...,6)
    for i in range(1, joint):
        #get z_(i-1) and o_(i-1)
        z_i_m1, o_i_m1 = z_i_0_lst[i-1], o_i_0_lst[i-1]
        s = getS(z_i_m1)

        #cross product of S(z_(i-1)) with (o_n - o_(i-1)) gives jv of i-th column/joint
        jv_i = s.dot(o_n - o_i_m1)
        #j_om of i-th column/joint
        j_om_i = z_i_m1

        #write the resulting combined jv_i, j_om_i into the i-th col of j_mat
        j_i = np.hstack((jv_i, j_om_i))
        j_mat[:,i-1] = j_i


    return j_mat



##
def getA(a, alp, d, theta):

    sin_theta, cos_theta = math.sin(theta), math.cos(theta)
    sin_alp, cos_alp = math.sin(alp), math.cos(alp)

    A = np.array([
                     [cos_theta, -sin_theta * cos_alp, sin_theta * sin_alp, a*cos_theta], #row1
                     [sin_theta, cos_theta * cos_alp, -cos_theta * sin_alp, a*sin_theta],  #row2
                     [0, sin_alp, cos_alp, d],
                     [0, 0, 0, 1]

                     ])

    return A



##
def getS(z_i_0):

    x, y, z = z_i_0[0], z_i_0[1], z_i_0[2]

    s = np.array([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0]
        ])

    return s



##
def checkUnconstr(body_v):

    unconstr_inds = []

    for i in range(len(body_v[0])):

        if( np.isnan(body_v[0][i]) ):

            unconstr_inds.append(i)

    return unconstr_inds
