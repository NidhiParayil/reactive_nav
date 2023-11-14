import time
import math
import random

import numpy as np
import pybullet as p
import pybullet_data


# from utilities import Models, Camera
from collections import namedtuple
# from attrdict import AttrDict
from tqdm import tqdm


# Blue is along -x and x -> first joint

class FailToReachTargetError(RuntimeError):
    pass


class MazeEnv():

    SIMULATION_STEP_DELAY = 1 / 240.

    def __init__(self, camera=None, vis=True):
        self.vis = vis
        if self.vis:
            self.p_bar = tqdm(ncols=0, disable=False)
        self.camera = camera
        
        # define environment
        self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        self.planeID = p.loadURDF("plane.urdf")
        self.agentID = p.loadURDF("/home/nidhi/reactive_nav/urdf/agent.urdf",flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT )
        numJoints = p.getNumJoints(self.agentID)
    
        jointInfo = namedtuple('jointInfo', 
            ['id','type','lowerLimit','upperLimit','maxForce','maxVelocity','controllable'])
        self.controllable_joints = []
        self.joints =[]
        for i in range(numJoints):
            info = p.getJointInfo(self.agentID, i)
            jointID = info[0]
            jointType = info[2]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = (jointType != p.JOINT_FIXED)
            if controllable:
                self.controllable_joints.append(jointID)
                p.setJointMotorControl2(self.agentID, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            info = jointInfo(jointID,jointType,jointLowerLimit,
                jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            self.joints.append(info)
        

        self.xin = p.addUserDebugParameter("x", -10, 10, 0.01)
        self.yin = p.addUserDebugParameter("y", -15, 5, .02)

    def debug_points(self, points):
        p.addUserDebugPoints([points],[[0,1,1]],5)

    def debug(self, points, colour,lineW):
        # p.addUserDebugPoints(points,colour,2)
        i =0
        for i in range(len(points)-1):
            p.addUserDebugLine(points[i], points[i+1], colour, lineWidth=lineW,lifeTime=0)
        p.addUserDebugLine(points[0], points[len(points)-1], colour, lineWidth=lineW,lifeTime=0)


    def debug_traj(self, points, colour,lineW):
        # p.addUserDebugPoints(points,colour,2)
        i =0
        for i in range(len(points)-1):
            p.addUserDebugLine(points[i], points[i+1], colour, lineWidth=lineW,lifeTime=0)


    def obs_state(self):
        joint1, joint2, joint3 = p.getJointStates(self.agentID,self.controllable_joints)
        self.obs_pos = [2,2.5+joint3[0],3]
        self.obs_polygon_edges = [[2-1.5,2.5+joint3[0]-.5,6],[2+1.5,2.5+joint3[0]-.5,6],[2+1.5,2.5+joint3[0]+.5,6],[2-1.5,2.5+joint3[0]+.5,6]]
        print(self.obs_polygon_edges)
        print("///////////////////")
        self.debug(self.obs_polygon_edges, [1,0,1],3)
        return self.obs_polygon_edges

    def step_simulation(self):
        p.stepSimulation()
        joint1, joint2, joint3 = p.getJointStates(self.agentID,self.controllable_joints)
        # print("joint states", joint1[0], joint2[0])
        if self.vis:
            time.sleep(self.SIMULATION_STEP_DELAY)
            self.p_bar.update(1)
        return joint1[0], joint2[0], joint3[0], joint1[1], joint2[1]

    def read_debug_parameter(self):
        x = p.readUserDebugParameter(self.xin)
        y = p.readUserDebugParameter(self.yin)
        return x, y


    def move_joints(self, x, y, dx, dy, force_cost):
        p.setJointMotorControl2(self.agentID, self.controllable_joints[0],targetPosition= x,controlMode= p.POSITION_CONTROL, force=force_cost)
        p.setJointMotorControl2(self.agentID, self.controllable_joints[1],targetPosition= y,controlMode=p.POSITION_CONTROL, force=force_cost)
    
    def step(self, x, y, force):
        x_, y_, obs, dx, dy =self.step_simulation()
        # x, y = self.read_debug_parameter()
        i=0
        for i in range(0,10):
            self.move_joints(x,y,dx, dy,force)
            i =i+1
        return x_, y_, obs

    def get_joint_states(self):
        j1, j2, _ = p.getJointStates(self.agentID, self.controllable_joints)
        return j1[0], j2[0]


    def detect_collision(self):
        self.step_simulation()
        return len(p.getContactPoints()) != 0

    def close(self):
        p.disconnect(self.physicsClient)


if __name__ == "__main__":
    maze = MazeEnv()
    i=-2
    while(True):
        maze.step(i)
        i = i+1
