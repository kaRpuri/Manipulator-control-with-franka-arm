import numpy as np
from math import pi, acos
from scipy.linalg import null_space
from copy import deepcopy

from lib.calculateFKJac import FK_Jac
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap


class PotentialFieldPlanner:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK_Jac()

    def __init__(self, tol=1e-4, max_steps=500, min_step_size=1e-5):
        """
        Constructs a potential field planner with solver parameters.

        PARAMETERS:
        tol - the maximum distance between two joint sets
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # YOU MAY NEED TO CHANGE THESE PARAMETERS

        # solver parameters
        self.tol = tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size


    ######################
    ## Helper Functions ##
    ######################
    # The following functions are provided to you to help you to better structure your code
    # You don't necessarily have to use them. You can also edit them to fit your own situation 

    @staticmethod
    def attractive_force(target, current, d = 0.2, zeta = 2.5):
        """
        Helper function for computing the attactive force between the current position and
        the target position for one joint. Computes the attractive force vector between the 
        target joint position and the current joint position 

        INPUTS:
        target - 3x1 numpy array representing the desired joint position in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame

        OUTPUTS:
        att_f - 3x1 numpy array representing the force vector that pulls the joint 
        from the current position to the target position 
        """

        ## STUDENT CODE STARTS HERE

        att_f = np.zeros((3, 1)) 
        if np.linalg.norm(current - target) > d: #use conic well
            att_f = -1*(current - target)/(np.linalg.norm(current-target))
        elif np.linalg.norm(current - target) <= d: #use parabolic well
            att_f = -zeta*(current - target)

        





        ## END STUDENT CODE

        return att_f

    @staticmethod
    def repulsive_force(obstacle, current, unitvec=np.zeros((3,1)), d = 0.8, eta = 0.001):
        """
        Helper function for computing the repulsive force between the current position
        of one joint and one obstacle. Computes the repulsive force vector between the 
        obstacle and the current joint position 

        INPUTS:
        obstacle - 1x6 numpy array representing the an obstacle box in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame
        unitvec - 3x1 numpy array representing the unit vector from the current joint position 
        to the closest point on the obstacle box 

        OUTPUTS:
        rep_f - 3x1 numpy array representing the force vector that pushes the joint 
        from the obstacle
        """

        ## STUDENT CODE STARTS HERE

        rep_f = np.zeros((3, 1)) 
        current = current.reshape(1, 3)
        rho, unit = PotentialFieldPlanner.dist_point2box(current, obstacle)
        
        unit = unit.reshape(3,1)

        if rho < 0:
            rep_f = 5            
        
        elif rho <= d and rho > 0:
            rep_f = -eta*((1/rho) - (1/d))*(1/rho**2)*unit
        else:
            rep_f = np.zeros((3, 1))


        ## END STUDENT CODE
        
        return rep_f

    @staticmethod
    def dist_point2box(p, box):
        """
        Helper function for the computation of repulsive forces. Computes the closest point
        on the box to a given point 
    
        INPUTS:
        p - nx3 numpy array of points [x,y,z]
        box - 1x6 numpy array of minimum and maximum points of box

        OUTPUTS:
        dist - nx1 numpy array of distance between the points and the box
                dist > 0 point outside
                dist = 0 point is on or inside box
        unit - nx3 numpy array where each row is the corresponding unit vector 
        from the point to the closest spot on the box
            norm(unit) = 1 point is outside the box
            norm(unit)= 0 point is on/inside the box

         Method from MultiRRomero
         @ https://stackoverflow.com/questions/5254838/
         calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        """
        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # Get box info
        boxMin = np.array([box[0], box[1], box[2]])
        boxMax = np.array([box[3], box[4], box[5]])
        boxCenter = boxMin*0.5 + boxMax*0.5
        p = np.array(p)

        # Get distance info from point to box boundary
        dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], np.zeros(p[:, 0].shape)]).T, 1)
        dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], np.zeros(p[:, 1].shape)]).T, 1)
        dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], np.zeros(p[:, 2].shape)]).T, 1)

        # convert to distance
        distances = np.vstack([dx, dy, dz]).T
        dist = np.linalg.norm(distances, axis=1)

        # Figure out the signs
        signs = np.sign(boxCenter-p)

        # Calculate unit vector and replace with
        unit = distances / dist[:, np.newaxis] * signs
        unit[np.isnan(unit)] = 0
        unit[np.isinf(unit)] = 0
        return dist, unit

    @staticmethod
    def compute_forces(target, obstacle, current):
        """
        Helper function for the computation of forces on every joints. Computes the sum 
        of forces (attactive, repulsive) on each joint. 

        INPUTS:
        target - 3x9 numpy array representing the desired joint/end effector positions 
        in the world frame
        obstacle - nx6 numpy array representing the obstacle box min and max positions
        in the world frame
        current- 3x9 numpy array representing the current joint/end effector positions 
        in the world frame

        OUTPUTS:
        joint_forces - 3x9 numpy array representing the force vectors on each 
        joint/end effector
        """

        ## STUDENT CODE STARTS HERE

        joint_forces = np.zeros((3, 9)) 
        n = np.shape(obstacle)[0]
        obstacle = np.reshape(obstacle, (-1, 6))
        

        for i in range(9):
            att_f = PotentialFieldPlanner.attractive_force(target[:,i], current[:,i])
            rep_f = np.zeros((3, 1))
            for j in range(n):
                rep_fo = PotentialFieldPlanner.repulsive_force(obstacle[j],current[:,i])
                rep_f +=rep_fo
            tot_f = att_f.reshape((3,1)) + rep_f.reshape((3,1))
    
        
            joint_forces[:,i] = tot_f.flatten()

        ## END STUDENT CODE
        
        return joint_forces
    
    @staticmethod
    def compute_torques(joint_forces, q):
        """
        Helper function for converting joint forces to joint torques. Computes the sum 
        of torques on each joint.

        INPUTS:
        joint_forces - 3x9 numpy array representing the force vectors on each 
        joint/end effector
        q - 1x7 numpy array representing the current joint angles

        OUTPUTS:
        joint_torques - 1x9 numpy array representing the torques on each joint 
        """

        ## STUDENT CODE STARTS HERE

        joint_torques = np.zeros((1, 9)) 
        j1,j2,j3,j4,j5,j6,j7,ee,jv8,jv9 = PotentialFieldPlanner.fk.compute_Ai(q)
        

        
        
        
        
        z0 = j1[:3, 2]
        z1 = j2[:3, 2]
        z2 = j3[:3, 2]
        z3 = j4[:3, 2]
        z4 = j5[:3, 2]
        z5 = j6[:3, 2]
        z6 = j7[:3, 2]
        z7 = jv8[:3, 2]
        z8 = jv9[:3, 2]

        j3o = [0,0,0.195,1]
        j5o = [0,0,0.125,1]
        j6o = [0,0,-0.015,1]
        j7o = [0,0,0.051,1]


        j3 = np.matmul(j3,j3o)
        j5 = np.matmul(j5,j5o)
        j6 = np.matmul(j6,j6o)
        j7 = np.matmul(j7,j7o)


        p0 = j1[:3, 3]
        p1 = j2[:3, 3]
        p2 = j3[:3]
        p3 = j4[:3, 3]
        p4 = j5[:3]
        p5 = j6[:3]
        p6 = j7[:3]
        pe = ee[:3, 3] 
        p7 = jv8[:3, 3]
        p8 = jv9[:3, 3]
    

        

        positions = [p0, p1, p2, p3, p4, p5, p6, p7, p8, pe]
        axes = [z0, z1, z2, z3, z4, z5, z6, z7, z8]
        
        torques = np.zeros((9,9))
        for i in range(9):
            J = np.zeros((3,9))
            for j in range(i+1):
                r = positions[i+1] - positions[j]  
                
                J[:,j] = np.cross(axes[j], r)
                
            
            torque = (J.T @ joint_forces[:,i])
            

            torques[i] = torque
        joint_torques = np.sum(torques, axis = 0)
        

        
        

        return joint_torques.reshape(1,9)




        ## END STUDENT CODE


    @staticmethod
    def q_distance(target, current):
        """
        Helper function which computes the distance between any two
        vectors.

        This data can be used to decide whether two joint sets can be
        considered equal within a certain tolerance.

        INPUTS:
        target - 1x7 numpy array representing some joint angles
        current - 1x7 numpy array representing some joint angles

        OUTPUTS:
        distance - the distance between the target and the current joint sets 

        """

        ## STUDENT CODE STARTS HERE

        distance = 0
        
        distance = np.linalg.norm((target - current))

        ## END STUDENT CODE

        return distance
    
    @staticmethod
    def compute_gradient(q, target, map_struct):
        """
        Computes the joint gradient step to move the current joint positions to the  
        next set of joint positions which leads to a closer configuration to the goal 
        configuration 

        INPUTS:
        q - 1x7 numpy array. the current joint configuration, a "best guess" so far for the final answer
        target - 1x7 numpy array containing the desired joint angles
        map_struct - a map struct containing the obstacle box min and max positions

        OUTPUTS:
        dq - 1x7 numpy array. a desired joint velocity to perform this task. 
        """

        ## STUDENT CODE STARTS HERE

        obstacle = map_struct.obstacles

        dq = np.zeros((1, 7))

        j1,j2,j3,j4,j5,j6,j7,ee,jv8,jv9 = PotentialFieldPlanner.fk.compute_Ai(q)
        
        j3o = [0,0,0.195,1]
        j5o = [0,0,0.125,1]
        j6o = [0,0,-0.015,1]
        j7o = [0,0,0.051,1]


        j3 = np.matmul(j3,j3o)
        j5 = np.matmul(j5,j5o)
        j6 = np.matmul(j6,j6o)
        j7 = np.matmul(j7,j7o)


        p0 = j1[:3, 3]
        p1 = j2[:3, 3]
        p2 = j3[:3]
        p3 = j4[:3, 3]
        p4 = j5[:3]
        p5 = j6[:3]
        p6 = j7[:3]
        pe = ee[:3, 3] 
        p7 = jv8[:3, 3]
        p8 = jv9[:3, 3]

        

       
    

        current = np.array([p1, p2, p3, p4, p5, p6, p7, p8, pe])

        j1,j2,j3,j4,j5,j6,j7,ee,jv8,jv9 = PotentialFieldPlanner.fk.compute_Ai(target)
        

        j3o = [0,0,0.195,1]
        j5o = [0,0,0.125,1]
        j6o = [0,0,-0.015,1]
        j7o = [0,0,0.051,1]


        j3 = np.matmul(j3,j3o)
        j5 = np.matmul(j5,j5o)
        j6 = np.matmul(j6,j6o)
        j7 = np.matmul(j7,j7o)


        p0 = j1[:3, 3]
        p1 = j2[:3, 3]
        p2 = j3[:3]
        p3 = j4[:3, 3]
        p4 = j5[:3]
        p5 = j6[:3]
        p6 = j7[:3]
        pe = ee[:3, 3] 
        p7 = jv8[:3, 3]
        p8 = jv9[:3, 3]
    

        

        

        target = np.array([p1, p2, p3, p4, p5, p6, p7, p8, pe])
        

        target = target.T
        current = current.T
        target = target.reshape(3,9)
        current = current.reshape(3,9)
        
        forces = PotentialFieldPlanner.compute_forces(target, obstacle, current)
        

        torques = PotentialFieldPlanner.compute_torques(forces,q)
        torques = torques[0,:-2]
        
        
        dq = torques/np.linalg.norm(torques)
        dq = dq.reshape(1,7)
        ## END STUDENT CODE
        
        
            
            
        return dq

    ###############################
    ### Potential Feild Solver  ###
    ###############################
    # def random_walk_around_collision(self, q, map_struct):
    #     """
    #     Execute a small random walk to avoid an obstacle, returning a safe configuration.
    #     """
    #     random_step = np.random.uniform(-0.1, 0.1, q.shape)
    #     q_new = q + random_step
    #     q_new = np.clip(q_new, self.lower, self.upper)

    #     # Repeat until finding a safe, collision-free configuration
    #     while detectCollision(q_new, map_struct):
    #         random_step = np.random.uniform(-0.1, 0.1, q.shape)
    #         q_new = q + random_step
    #         q_new = np.clip(q_new, self.lower, self.upper)

    #     return q_new

    # def random_walk_minima(self, q, map_struct):
    #     perturbation = np.random.uniform(-0.1, 0.1, q.shape)
    #     q_new = q + perturbation
    #     q_new = np.clip(q_new, self.lower, self.upper)

    #     # Ensure perturbation avoids collision
    #     if detectCollision(q_new, map_struct):
    #         q_new = self.random_walk_around_collision(q, map_struct)

    #     return q_new



    def linkcollision(self, q, map_struct):


        jointpos, _ = self.fk.forward_expanded(q.flatten())

        startIndices = [0,1,2,3,4,5,6]
        endIndicies = [1,2,3,4,5,6,7]

        startPoints = jointpos[startIndices, ::]
        endPoints = jointpos[endIndicies, ::]

        collided = False
        for obstacle in map_struct.obstacles:
            if any(detectCollision(startPoints, endPoints, obstacle)):
                collided = True

        return collided
    

    def pathcollision(self, q, q_new, map_struct):
        currentjointpos, _ = self.fk.forward_expanded(q.flatten())
        newjointpos, _ = self.fk.forward_expanded(q_new.flatten())

        collided = False
        for obstacle in map_struct.obstacles:
            if any(detectCollision(currentjointpos, newjointpos, obstacle)):
                collided = True

        return collided
    
    def localminimacheck(self, q_path, window_size=10, distance_threshold=0.005):
    
        if len(q_path) < window_size:
            return False  # Not enough data to determine local minima

        recent_path = q_path[-window_size:]
        
        # Calculate the total distance moved in the recent path
        total_distance = 0
        for i in range(1, len(recent_path)):
            total_distance += self.q_distance(recent_path[i], recent_path[i-1])
        
        # Calculate the direct distance between start and end of the recent path
        direct_distance = self.q_distance(recent_path[0], recent_path[-1])
        
        # If the total distance is significantly larger than the direct distance,
        # and the direct distance is very small, we're likely in a local minima
        if total_distance > 2 * direct_distance and direct_distance < distance_threshold:
            return True
        
        return False
    

    def random_walk(self, q, map_struct, num_steps=5, step_size=0.1):
    
        q_current = q.flatten()
        valid_steps = 0
        
        while valid_steps < num_steps:
            # Generate a random direction within the joint limits
            random_direction = np.random.uniform(self.lower, self.upper) - q_current
            random_direction /= np.linalg.norm(random_direction)
            
            # Take a step in the random direction
            q_new = q_current + step_size * random_direction
            q_new = np.clip(q_new, self.lower, self.upper)
            
            # Check for collisions
            path_collided = self.pathcollision(q_current.reshape(1, 7), q_new.reshape(1, 7), map_struct)
            link_collided = self.linkcollision(q_new.reshape(1, 7), map_struct)
            
            if not path_collided and not link_collided:
                q_current = q_new
                valid_steps += 1
            
            # If collision occurs, the loop will continue without incrementing valid_steps
        
        return q_current.reshape(1, 7)



    def plan(self, map_struct, start, goal):
        """
        Uses potential field to move the Panda robot arm from the startng configuration to
        the goal configuration.

        INPUTS:
        map_struct - a map struct containing min and max positions of obstacle boxes 
        start - 1x7 numpy array representing the starting joint angles for a configuration 
        goal - 1x7 numpy array representing the desired joint angles for a configuration

        OUTPUTS:
        q - nx7 numpy array of joint angles [q0, q1, q2, q3, q4, q5, q6]. This should contain
        all the joint angles throughout the path of the planner. The first row of q should be
        the starting joint angles and the last row of q should be the goal joint angles. 
        """

        q_path = np.array([start])
        T = 0
        stagnte_count = 0
        while T < self.max_steps:
           
            alpha = 1.0/(T+1)

            ## STUDENT CODE STARTS HERE
            
            # The following comments are hints to help you to implement the planner
            # You don't necessarily have to follow these steps to complete your code 
            
            # Compute gradient 
            # TODO: this is how to change your joint angles 
            

            # q = q_path[-1]
            q = q_path[-1]
            dq = self.compute_gradient(q, goal, map_struct)
            

            

            
            # q = q_path[-1].reshape(1,7)
            q_new = q_path[-1].reshape(1,7) + alpha*dq
            q_new = np.clip(q_new, self.lower,self.upper)
            
            
            newjointpos, _ = self.fk.forward_expanded(q_new.flatten())
            latestjointpos, _ = self.fk.forward_expanded(q_path[-1].reshape(1,7).flatten())
            # startjointpos = startjointpos.T.reshape(3,10)
            # nowjointpos = nowjointpos.T.reshape(3,10)

            pathcollided = self.pathcollision(q, q_new, map_struct)
            if pathcollided:
                print("pathcollision: ", T)
            linkcollided = self.linkcollision(q_new, map_struct)
            if linkcollided:
                print("linkcollision: ", T)            
            minima = self.localminimacheck(q_path)
            if minima == True:
                print("localminima ist true: ", T)

            
            if pathcollided or linkcollided or minima:
                q_arw = self.random_walk(q, map_struct)
                print("attempting a random walk: ", q_arw)
                q_new = q_arw


            # if self.q_distance(q,q_path[-1].reshape(1,7)) < 0.0000001:
            #     stagnte_count+=1
            # if stagnte_count == 10:
            #     print("random walk done")
            #     q = q_path[-1].reshape(1,7) + np.random.uniform(-0.05, 0.05, size=q_path[-1].reshape(1,7).shape)
            #     q = np.clip(q, self.lower, self.upper)
            #     stagnte_count = 0


        
            
            # if is_collision:
            #     # print("collision detected")
            #     q = q_path[-1].reshape(1,7) + np.random.uniform(-0.05, 0.05, size=q_path[-1].reshape(1,7).shape)
            #     dq = self.compute_gradient(q.flatten(), goal, map_struct)
            #     q = q + 0.1
            #     q = np.clip(q, self.lower, self.upper)




            
            
            # Termination Conditions
            if self.q_distance(q_path[-1], goal) <= self.min_step_size: # TODO: check termination conditions
                break # exit the while loop if conditions are met!

           

                
            
            
            ## END STUDENT CODE
            
            q_path = np.vstack((q_path, q_new))
           
            T+=1

        return q_path

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    planner = PotentialFieldPlanner()
    
    # inputs 
    from pathlib import Path
    maps_dir = Path(__file__).resolve().parents[2] / "maps"
    map_struct = loadmap(str(maps_dir / "map1.txt"))
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    
    # potential field planning
    q_path = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print(q_path)
    # show results
    for i in range(q_path.shape[0]):
        error = PotentialFieldPlanner.q_distance(q_path[i, :], goal)
        print('iteration:',i,' q =', q_path[i, :], ' error={error}'.format(error=error))

    print("q path: ", q_path)
    print("q_path.shape: ", q_path.shape)

