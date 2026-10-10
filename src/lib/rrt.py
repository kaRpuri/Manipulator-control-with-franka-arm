import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy

from lib.calculateFK import FK

FK = FK()

def checkObstacleCollisions(obstacles, currentConfig):
    '''
    Checks if there are obstacle collisions for a config currentConfig
    '''
    collided = False

    currentJointPos, currentTransform = FK.forward(q=currentConfig)
    startIndices = [0,1,2,3,4,5,6]
    endIndicies = [1,2,3,4,5,6,7]

    startPoints = currentJointPos[startIndices, ::]
    endPoints = currentJointPos[endIndicies, ::]

    for obstacle in obstacles:
        if any(detectCollision(startPoints, endPoints, obstacle)):
            collided = True
            #print("detected obstacle collision")

    return collided

def checkSelfCollisions(currentConfig):
    '''
    Checks if there are self collisions for a config currentConfig
    '''
    collided = False

    currentJointPos, currentTransform = FK.forward(q=currentConfig)
    #Only check joints farther out for self collision!
    startIndices = [4,5,6]
    endIndicies = [5,6,7]

    startPoints = currentJointPos[startIndices, ::]
    endPoints = currentJointPos[endIndicies, ::]

    #Create a box around the base of the robot. If the robot is in this area it is probably a self collision
    robotBox = np.array([-0.125, -0.125, 0, 0.125, 0.125, 0.5])
    if any(detectCollision(startPoints, endPoints, robotBox)):
        #print("detected self collision")
        collided = True

    return collided

def findClosestNode(tree, newNode):
    '''
    Finds the node closest to newNode in tree
    '''
    nodeDistances = []
    for node in tree:
        nodeDistances.append(np.linalg.norm(newNode - node))
    closestNode = nodeDistances.index(min(nodeDistances))
    #print(closestNode)
    return tree[closestNode]

def pathCollision(newNode, treeNode, obstacles):
    '''
    Checks if there are collisions along a path from newNode to treeNode with obstacles
    or if there is a self-collision
    '''
    pathCollided = False
    pathNumSteps = 100
    currentPathPosition = newNode

    #print("newnode")
    #print(newNode)
    #print("treenode")
    #print(treeNode)
    

    for i in range(pathNumSteps):
        positionIncrement = ((treeNode - newNode)/pathNumSteps)
        currentPathPosition = newNode + (i+1)*positionIncrement
        #print("path position")
        #print(currentPathPosition)
        if checkObstacleCollisions(obstacles, currentPathPosition):
            pathCollided = True
        if checkSelfCollisions(currentPathPosition):
            pathCollided = True
        
    return pathCollided


def findNodeIndex(tree, node):
    '''
    Finds the index of the node `node` in `tree`.
    '''
    for i, n in enumerate(tree):
        if np.array_equal(n, node):
            return i
    return 0

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """


    # initialize path
    path = []

    #Initialize Safety Factor (use 0.1 meters)
    safetyFactor = 0.1

    #Max number of iterations
    maxIterations = 500

    # get joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    #Load Current Obstacles
    obstacles = map.obstacles
    print("Obstacles in Map")
    print(obstacles)

    safetyZones = []

    #Increase Safety Zone Around Obstacles Due To Robot Size.
    safetyAddition =  ([-safetyFactor, -safetyFactor, -safetyFactor, safetyFactor, safetyFactor, safetyFactor])
    safetyZones = np.array([(safetyAddition + obstacle) for obstacle in obstacles])

    print("Safety Zones")
    print(safetyZones)

    #Check that start and goal configs are within joint limits
    if (start<lowerLim).any() or (start>upperLim).any() or (goal<lowerLim).any() or (goal>upperLim).any():
        print("invalid start or goal")
        return path
    
    #Check that start and goal do not cause collisions
    if checkObstacleCollisions(safetyZones, start):
        print("collided at start")
        return path
    if checkObstacleCollisions(safetyZones, goal):
        print("goal results in collision")
        return path
    if checkSelfCollisions(goal):
        print("goal results in self collision")
        return path
    if checkSelfCollisions(start):
        print("start results in self collision")
        return path
    
    #initialize start and goal tree
    startTree = [start]
    goalTree = [goal]
    startConnectToIndex = [-5]
    goalConnectToIndex = [-5]
    currentStartIndex = 0
    currentGoalIndex = 0
    completedPath = False
    fullPath = []
    #Begin Loop to calculate positions
    for iteration in range(maxIterations):
        validSample = False
        
        #Loop until we find a valid point to sample
        while not validSample:
            randomConfig = np.random.uniform(lowerLim, upperLim)
            #print(randomConfig)
            obstacleCollision = checkObstacleCollisions(safetyZones, randomConfig)
            selfCollision = checkSelfCollisions(randomConfig)

            if not obstacleCollision and not selfCollision:
                #print("not collided")
                validSample = True
        
        #Find closest node in start tree and the closest node in the end tree
        closestStartNode = findClosestNode(startTree, randomConfig)
        #print(closestStartNode)

        closestEndNode = findClosestNode(goalTree, randomConfig)
        #print(closestEndNode)

        #Then check to see if there is a collision free path to this node from the closest node
        #in each tree
        validStartTreePath = False
        validGoalTreePath = False
        if not pathCollision(randomConfig, closestStartNode, safetyZones):
            validStartTreePath = True
            print("valid start tree path")
        
        if not pathCollision(randomConfig, closestEndNode, safetyZones):
            validGoalTreePath = True
            print("valid goal tree path")


        #if yes to both, this is the final node, add to both trees, and return path.
        if(validGoalTreePath and validStartTreePath):
            startTree.append(randomConfig)
            startConnectToIndex.append(currentStartIndex)
            goalConnectToIndex.append(currentGoalIndex)
            currentStartIndex = currentStartIndex + 1
            currentGoalIndex = currentGoalIndex +1
            #goalTree.append(randomConfig)
            completedPath = True
            print("Path found, breaking loop")
            #print(goalTree)
            break

        #if yes to start tree, add to start tree.
        if(validGoalTreePath and not validStartTreePath):
            goalTree.append(randomConfig)
            currentGoalIndex = findNodeIndex(goalTree, closestEndNode)
            goalConnectToIndex.append(currentGoalIndex)


        #if yes to goal tree, add to goal tree.
        if(validStartTreePath and not validGoalTreePath):
            startTree.append(randomConfig)
            currentStartIndex = findNodeIndex(startTree, closestStartNode)
            startConnectToIndex.append(currentStartIndex)


        #if no to both, go back to beginning and find a new random node

    #If we got a completed path, loop back through the path from connect point to finish
    #to get the actual path.
    #We need to only add those nodes to the final path that are along the path, not all nodes
    print("loop broken")
    print("path connected through start:")
    print(startConnectToIndex)
    print("and end:")
    print(goalConnectToIndex)

    if completedPath:
        indexConnectedTo = startConnectToIndex[len(startConnectToIndex)-1]
        print(indexConnectedTo)
        path.append(startTree[len(startTree)-1])
        
        #Going through start tree
        while indexConnectedTo != -5:
            path.append(startTree[indexConnectedTo])
            indexConnectedTo = startConnectToIndex[indexConnectedTo]
            print(indexConnectedTo)
            if(indexConnectedTo == -1):
                break

        #REVERSE array so start comes first
        path = path[::-1]

        #Go thru goal tree
        indexConnectedTo = goalConnectToIndex[len(goalConnectToIndex)-1]
        print(indexConnectedTo)

        while indexConnectedTo != -5:
            path.append(goalTree[indexConnectedTo])
            indexConnectedTo = goalConnectToIndex[indexConnectedTo]
            print(indexConnectedTo)
            if(indexConnectedTo == -1):
                break

        '''OLD CODE THAT DIDN'T FIND PATH CORRECTLY
        path = startTree
        i = len(goalTree)-1
        while i >= 0:
            path.append(goalTree[i])
            i = i - 1
        print("path complete!")
        #print(path)'''
    else:
        print("no path found after max iterations")
        return path

    return np.array(path)

if __name__ == '__main__':
    from pathlib import Path
    maps_dir = Path(__file__).resolve().parents[2] / "maps"
    map_struct = loadmap(str(maps_dir / "map1.txt"))
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print(path)
    
