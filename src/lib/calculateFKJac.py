import numpy as np
from math import pi

class FK_Jac():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab 1 and 4 handout

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
    


    def forward_expanded(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -10 x 3 matrix, where each row corresponds to a physical or virtual joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 10 x 4 x 4 homogeneous transformation matrix,
                  representing the each joint/end effector frame expressed in the
                  world frame
        """

        # Your code starts here

        jointPositions = np.zeros((10,3))
        T0e = np.zeros((10,4,4))

        j3o = [0,0,0.195,1]
        j5o = [0,0,0.125,1]
        j6o = [0,0,-0.015,1]
        j7o = [0,0,0.051,1]


        he8 = np.array(([1,0,0,0],
                       [0,1,0,-0.100],
                       [0,0,1,-0.105],
                       [0,0,0,1]))
        he9 = np.array(([1,0,0,0],
                       [0,1,0,0.100],
                       [0,0,1,-0.105],
                       [0,0,0,1]))


        Az1,A12,A23,A34,A45,A56,A67,A7e = self.panda_dh(q)
        j1 = Az1
        j2 = np.matmul(j1,A12)
        j3 = np.matmul(j2,A23)
        j4 = np.matmul(j3,A34)
        j5 = np.matmul(j4,A45)
        j6 = np.matmul(j5,A56)
        j7 = np.matmul(j6,A67)
        ee = np.matmul(j7,A7e)
        jv8 = np.matmul(ee, he8)
        jv9 = np.matmul(ee, he9)





        
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
        jointPositions[9, :3] = ee[0:3,-1]
        jointPositions[7, :3] = jv8[0:3,-1]
        jointPositions[8, :3] = jv9[0:3,-1]

        T0e[0] = j1
        T0e[1] = j2
        T0e[2] = j3
        T0e[3] = j4
        T0e[4] = j5
        T0e[5] = j6
        T0e[6] = j7
        T0e[7] = ee
        T0e[8] = jv8
        T0e[9] = jv9




        

        

        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE

        Az1,A12,A23,A34,A45,A56,A67,A7e = self.panda_dh(q)
        j1 = Az1
        j2 = np.matmul(j1,A12)
        j3 = np.matmul(j2,A23)
        j4 = np.matmul(j3,A34)
        j5 = np.matmul(j4,A45)
        j6 = np.matmul(j5,A56)
        j7 = np.matmul(j6,A67)
        ee = np.matmul(j7,A7e)
        he8 = np.array(([1,0,0,0],
                       [0,1,0,-0.100],
                       [0,0,1,-0.105],
                       [0,0,0,1]))
        he9 = np.array(([1,0,0,0],
                       [0,1,0,0.100],
                       [0,0,1,-0.105],
                       [0,0,0,1]))
        jv8 = np.matmul(ee, he8)
        jv9 = np.matmul(ee, he9)
        

        
        return j1,j2,j3,j4,j5,j6,j7,ee,jv8,jv9

        
    
if __name__ == "__main__":

    fk = FK_Jac()

    # matches figure in the handout
    q = np.array([0,-1,0,-2,0,1.57,0])
    

    joint_positions, T0e = fk.forward_expanded(q)
    j1,j2,j3,j4,j5,j6,j7,ee,jv8,jv9 = fk.compute_Ai(q)

    jays = np.array([j1,j2,j3,j4,j5,j6,j7,ee,jv8,jv9])
    print(j1[:3,3])
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
