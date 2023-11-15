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
        self.control_type = "POSITION"
        jointInfo = namedtuple('jointInfo', 
            ['id','type','controllable'])
        self.controllable_joints = []
        self.joints =[]
        for i in range(numJoints):
            info = p.getJointInfo(self.agentID, i)
            jointID = info[0]
            jointType = info[2]
            # jointLowerLimit = info[8]
            # jointUpperLimit = info[9]
            # jointMaxForce = info[10]
            # jointMaxVelocity = info[11]
            controllable = (jointType != p.JOINT_FIXED)
            if controllable:
                self.controllable_joints.append(jointID)
                p.setJointMotorControl2(self.agentID, jointID, p.VELOCITY_CONTROL, force=0.0)
                p.enableJointForceTorqueSensor(self.agentID, jointID)
            info = jointInfo(jointID,jointType,controllable)
            self.joints.append(info)
        

        self.xin = p.addUserDebugParameter("x", -10, 10, 0.01)
        self.yin = p.addUserDebugParameter("y", -15, 5, .02)
        self.f1 =0
        self.f2 =0
        self.force_flag = 0

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

    # def step_si(self):
        
        
    #     # print("joint states", joint1[0], joint2[0])
    #     if self.vis:
    #         time.sleep(self.SIMULATION_STEP_DELAY)
    #         self.p_bar.update(1)
    #     return 

    def read_debug_parameter(self):
        x = p.readUserDebugParameter(self.xin)
        y = p.readUserDebugParameter(self.yin)
        return x, y


    def move_joints_pos_control(self, x, y):
        print("position")
        p.setJointMotorControl2(self.agentID, self.controllable_joints[0],targetPosition= x,controlMode= p.POSITION_CONTROL)
        p.setJointMotorControl2(self.agentID, self.controllable_joints[1],targetPosition= y,controlMode=p.POSITION_CONTROL)
    
    def move_joints_admittance_controller(self, px, py, vx, vy, f1, f2):
        print("admittance")
        # p.setJointMotorControl2(self.agentID, self.controllable_joints[0], p.TORQUE_CONTROL, force=10)
        p.setJointMotorControl2(self.agentID, self.controllable_joints[1], p.VELOCITY_CONTROL, force=0.0)
        p.setJointMotorControl2(self.agentID, self.controllable_joints[1], p.TORQUE_CONTROL, force=f2)
        p.setJointMotorControl2(self.agentID, self.controllable_joints[0], p.VELOCITY_CONTROL, force=0.0)
        p.setJointMotorControl2(self.agentID, self.controllable_joints[0], p.TORQUE_CONTROL, force=f1)



    def step(self, x, y):
        p.stepSimulation()
        joint1, joint2, joint3 = p.getJointStates(self.agentID,self.controllable_joints)
        px, py, pobs = joint1[0], joint2[0], joint3[0]
        vx, vy, vobs = joint1[1], joint2[1], joint3[1]
        # fx, fy, fobs = joint1[2], joint2[2], joint3[2]
        values = p.getContactPoints(self.agentID)
        for val in values:
            force = val[9]
        #     print(val)
        #     print(force)
        # print(force)

        # print(fx, fy, self.control_type)
        # if any(np.abs(value) > 0 for value in fy):
        if np.abs(force)>0.5:
            self.control_type = "ADMITTANCE"
        else:
            self.control_type = "POSITION"
        
        if self.control_type == "ADMITTANCE":
            self.f1 = 0
            self.f2 = force*100
            self.move_joints_admittance_controller(px, py, vx, vy, self.f1, self.f2)
            self.force_flag = self.force_flag+1
            if self.force_flag > 10:
                self.control_type = "POSITION"
                self.move_joints_pos_control(x,y)
                self.force_flag =0
            
        else:
            self.move_joints_pos_control(x,y)
            
        # self.move_joints_pos_control(x,y)
        # self.move_joints_admittance_controller(px, py, vx, vy, fx, fy)

            
        time.sleep(self.SIMULATION_STEP_DELAY*10)
        return px, py, pobs

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
