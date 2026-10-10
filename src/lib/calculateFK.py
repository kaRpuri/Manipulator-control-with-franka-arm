import numpy as np
from math import pi

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        pass

    def dh_matrix(self, a,alpha,d,theta):

        c_theta = np.cos(theta)
        c_alpha = np.cos(alpha)
        s_theta = np.sin(theta)
        s_alpha = np.sin(alpha)

        A = np.array([
        [c_theta, -s_theta * c_alpha,  s_theta * s_alpha, a * c_theta],
        [s_theta,  c_theta * c_alpha, -c_theta * s_alpha, a * s_theta],
        [0,        s_alpha,            c_alpha,            d],
        [0,        0,                  0,                  1]
    ])
        
        return A

    def panda_dh(self, q):

        Az1 = self.dh_matrix(0, 0, 0.141, 0)
        A12 = self.dh_matrix(0, -pi/2, 0.192, q[0])
        A23 = self.dh_matrix(0, pi/2, 0, q[1])
        A34 = self.dh_matrix(0.0825, pi/2, 0.195+0.121, q[2])
        A45 = self.dh_matrix(0.0825, pi/2, 0, q[3]+pi)
        A56 = self.dh_matrix(0, -pi/2, 0.125+0.259, q[4])
        A67 = self.dh_matrix(0.088, pi/2, 0, q[5]-pi)
        A7e = self.dh_matrix(0, 0, 0.051+0.159, q[6]-pi/4)
        return Az1,A12,A23,A34,A45,A56,A67,A7e



    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)


        #defining coordinate offsets
        j3o = [0,0,0.195,1]
        j5o = [0,0,0.125,1]
        j6o = [0,0,-0.015,1]
        j7o = [0,0,0.051,1]



        Az1,A12,A23,A34,A45,A56,A67,A7e = self.panda_dh(q)
        j1 = Az1
        j2 = np.matmul(j1,A12)
        j3 = np.matmul(j2,A23)
        j4 = np.matmul(j3,A34)
        j5 = np.matmul(j4,A45)
        j6 = np.matmul(j5,A56)
        j7 = np.matmul(j6,A67)
        ee = np.matmul(j7,A7e)
        
        j3r = np.matmul(j3,j3o)
        j5r = np.matmul(j5,j5o)
        j6r = np.matmul(j6,j6o)
        j7r = np.matmul(j7,j7o)

        jointPositions[0] = j1[0:3,-1]
        jointPositions[1, :3] = j2[0:3,-1]
        jointPositions[2, :3] = j3r[0:3]
        jointPositions[3, :3] = j4[0:3,-1]
        jointPositions[4, :3] = j5r[0:3]
        jointPositions[5, :3] = j6r[0:3]
        jointPositions[6, :3] = j7r[0:3]
        jointPositions[7, :3] = ee[0:3,-1]

        T0e = ee



        # Your code ends here

        return jointPositions, T0e



    # feel free to define additional helper methods to modularize your solution for lab 1

    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        Zs = np.zeros((3,7))
        j1,j2,j3,j4,j5,j6,j7,ee = self.compute_Ai(q)
        zo = np.array([0, 0, 1])  # z-axis of the base frame
        Zs[0] = j1[:3, 2]
        Zs[1] = j2[:3, 2]
        Zs[2] = j3[:3, 2]
        Zs[3] = j4[:3, 2]
        Zs[4] = j5[:3, 2]
        Zs[5] = j6[:3, 2]
        Zs[6] = j7[:3, 2]
        # STUDENT CODE HERE: This is a function needed by lab 2

        return 
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2
        Az1,A12,A23,A34,A45,A56,A67,A7e = self.panda_dh(q)
        j1 = Az1
        j2 = np.matmul(j1,A12)
        j3 = np.matmul(j2,A23)
        j4 = np.matmul(j3,A34)
        j5 = np.matmul(j4,A45)
        j6 = np.matmul(j5,A56)
        j7 = np.matmul(j6,A67)
        ee = np.matmul(j7,A7e) 
        return j1,j2,j3,j4,j5,j6,j7,ee
        

        
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,-pi/2,pi/2,pi/2,pi/4])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
