import numpy as np
from lib.calculateFK import FK



# import numpy as np
# from lib.calculateFK import FK



def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    ## STUDENT CODE GOES HERE
    joints = FK()
    j1,j2,j3,j4,j5,j6,j7,ee = joints.compute_Ai(q_in)

    po = np.array([0, 0, 0])  # origin
    p0 = j1[:3, 3]
    p1 = j2[:3, 3]
    p2 = j3[:3, 3]
    p3 = j4[:3, 3]
    p4 = j5[:3, 3]
    p5 = j6[:3, 3]
    p6 = j7[:3, 3]
    pe = ee[:3, 3] 
    
    zo = np.array([0, 0, 1])  # z-axis of the base frame
    z0 = j1[:3, 2]
    z1 = j2[:3, 2]
    z2 = j3[:3, 2]
    z3 = j4[:3, 2]
    z4 = j5[:3, 2]
    z5 = j6[:3, 2]
    z6 = j7[:3, 2]

    J[:3, 0] = np.cross(z0, pe-p0)
    J[3:, 0] = z0

    J[:3, 1] = np.cross(z1, pe-p1)
    J[3:, 1] = z1

    J[:3, 2] = np.cross(z2, pe-p2)
    J[3:, 2] = z2

    J[:3, 3] = np.cross(z3, pe-p3)
    J[3:, 3] = z3

    J[:3, 4] = np.cross(z4, pe-p4)
    J[3:, 4] = z4

    J[:3, 5] = np.cross(z5, pe-p5)
    J[3:, 5] = z5

    J[:3, 6] = np.cross(z6, pe-p6)
    J[3:, 6] = z6

    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, 0,0,0,0])
    print(np.round(calcJacobian(q),3))

    


