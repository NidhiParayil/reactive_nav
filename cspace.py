
from os.path import abspath, dirname, join
import random
import sys
sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings'))
from ompl import util as ou
from ompl import base 
from ompl import geometric

class StateValidityCheckerFn(base.StateValidityChecker):
    def __init__(self, si):
        super(StateValidityCheckerFn, self).__init__(si)

    def isValid(self, state):
        return True



class StateSampler:


    def __init__(self, num_states = 2, num_samples=50):
        self.space = base.RealVectorStateSpace(num_states) 
        # Set bounds for the state space
        self.bounds = base.RealVectorBounds(num_states)
        self.bounds.setLow(0)  # Lower bound for position and velocity
        self.bounds.setHigh(5)  # Upper bound for position and velocity
        self.space.setBounds(self.bounds)
        self.ss =geometric.SimpleSetup(self.space)
        # self.ss.setStateValidityChecker(lambda state: isStateValid(state))
        self.num_samples = num_samples
        self.joint_1_lim =[0,5]
        self.joint_2_lim =[0,5]
        self.max_joint_vel = 2
        state_checker = StateValidityCheckerFn(self.ss.getSpaceInformation())

        # Set the state validity checker
        self.ss.setStateValidityChecker(state_checker)
        
        self.pos, self.v = self.get_sample(0,5)
        self.start_state= [0,0]
        # self.goal_state= [self.x[random.randint(0, len(self.x)-1)],self.y[random.randint(0, len(self.y)-1)],0,0]
        self.goal_state =[4.75, 4.75]
  
     # Generate a sample in the valid part of the R^3 state space.
     # Valid states satisfy the following constraints:
     # -1<= x,y,z <=1
     # if .25 <= z <= .5, then |x|>.8 and |y|>.8
    def get_sample(self, low, high):
        self.bounds.setLow(low)  # Lower bound for position and velocity
        self.bounds.setHigh(high)  # Upper bound for position and velocity
        self.states_pos =[]
        self.states_vel =[]
        for i in range(self.num_samples):
            state = self.space.allocState()
            self.ss.getSpaceInformation().allocStateSampler().sampleUniform(state)
            self.states_pos.append([round(state[0],3),round(state[1],3)])
            self.states_vel.append([round(state[2],3),round(state[4],3)])
        return self.states_pos, self.states_vel


    def get_new_state(self, visited_states):
        
        while(len(visited_states)!=0):
            ran_val = random.randint(0, len(self.pos)-1)
            ran_vel = random.randint(0, len(self.v)-1)
            new_state = [self.pos[ran_val][0],self.pos[ran_val][1],self.v[ran_vel][0],self.v[ran_vel][0]]
            # print("visited state", visited_states)
            if new_state[:2] not in visited_states[:][0:2]:
                return new_state
        
        return new_state
