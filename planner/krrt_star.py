import random
import numpy as np
import time
import sys
sys.path.append('/home/nidhi/pybullet_ws/planner/')
from planners import Planner
sys.path.append('/home/nidhi/pybullet_ws/')
from scripts import optimal_control 


def get_trajectory(state1, state2, time=5):
    # x = a0+a1*t+a2*t^2+a3*t^3
    # v = a1+2*a2*t+3*a3*t^2
    # a = 2*a2+6*a3*t
    # a0 = x, a1 = v
    a3_x = ((state2[2]-state1[2])/(time^2)-(2*(state2[0]-state1[0])/(time^3)))
    a2_x = (state2[2]-state1[2])/(time*2) - (3*time*a3_x/2)
    a3_y = ((state2[3]-state1[3])/(time^2)-(2*(state2[1]-state1[1])/(time^3)))
    a2_y = (state2[3]-state1[3])/(time*2) - (3*time*a3_y/2)
    acc_x = 2*a2_x+6*a3_x*time
    acc_y = 2*a2_y+6*a3_y*time
    return acc_x, acc_y


def cal_cost(state1, state2, ax,ay, control_force, obs,sensed_force =0):
    dist = np.sqrt((state1[0]-state2[0])**2 +(state1[1]-state2[1])**2 )
    if obs == True:
        dist = 20
        
    else:
        reaction_force = 0

    # force = (control_force*.01+reaction_force)
    
    cost = dist
    # print("distance", dist)

    # if ax != 0:
    #     timex = (-state1[2]+ np.sqrt(state1[2]+np.abs(state2[0]-state1[0])*2*ax))/(2*ax)
    # else:
    #     timex = np.abs(state2[0]-state1[0]) / np.abs(state2[2]-state1[2])
    # if np.isnan(timex):
    #     timex=0
    # if ay != 0:
    #     timey = (-state1[3]+ np.sqrt(state1[3]+np.abs(state2[1]-state1[1])*2*ay))/(2*ay)
    # else:
    #     timey = np.abs(state2[1]-state1[1]) / np.abs(state2[3]-state1[3])
    # if np.isnan(timey):

    #     timey=0
    # if timey != timex: 
    #     time = max(timex, timey)
    # else:
    #     time = 0
    # cost =  force+time 
    return cost, control_force*.1

def least_cost_traj(costs):
    parent_id = costs.index(min(costs))
    cost = min(costs)
    return parent_id, cost


class krrt(Planner):

    def __init__(self):
        print("initialising krrt")
        self.existing_states = []
        self.curr_state = None
        self.prev_state = None

        A = np.asmatrix([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [1, 2, 1, 0]])  # Example A matrix
        B = np.asmatrix([0,0,1,1]).T  # Example B matrix

        # Define the weight matrices Q and R for the 4-state system
        Q = np.asmatrix([[1, 0, 0, 0], [0, 2, 0, 0], [0, 0, 3, 0], [0, 0, 0, 4]])  # Example weight matrix for states
        R = np.asmatrix([[1]])  # Example weight matrix for control inputs


        self.opt = optimal_control.OptimalControl(A, B, Q, R)
        # opt.get_optimal_control()
        # x0 = np.asmatrix([0,1,0,1]).T
        # x1= np.asmatrix([2,2,0,1]).T
        # opt_time, cost = self.opt.get_optimal_time(x0,x1)


    def convert_key(self, key):
        key_ = [s.strip() for s in key.split(',')]
        key_1= [i for i in key_[0]]
        key_2= [i for i in key_[-1]]
        val =''
        for i in key_1:
            if i!='[':
                val =val+i
        key_[0] = val
        val_ =''
        for i in key_2:
            if i!=']':
                val_ =i+val_
        key_[-1] = val_
                    
        for c in range(0,len(key_)):
            key_[c] = float(key_[c] )
        return key_

    def detect_obs(self, obs_edges):
        # min_x =  min(obs_edges[:][0])
        min_y =  min(obs_edges[:][1])
        # max_x =  max(obs_edges[:][0])
        max_y =  max(obs_edges[:][1])

        # x1 = self.curr_state[0]-1.5
        # x2 = self.curr_state[0]+1.5
        y2 = self.curr_state[1]+1.5        

        if y2>min_y and y2<max_y:
            
            return True
        else:
            return False

    def run_planner(self,cspace, ob_states):
        start_time = time.time()
        self.existing_states = {}
        self.curr_state = cspace.start_state
        cost = 0
        self.existing_states[str(self.curr_state)] ={
                                                'parent':None,
                                                'cost_p':0,
                                                'cost_tot':0
                                                }
        self.visited_states =[]
        self.visited_states.append(self.curr_state)
        run = True
        obstacle_encountered = False
        while (run):
            end_time = time.time()
            # print("in while loop")
            if (end_time-start_time> .020):
                run = False
                cspace.goal_state = self.curr_state
            rand_state = cspace.get_new_state(self.visited_states)
            
            self.curr_state = rand_state
            obstacle_encountered = self.detect_obs(ob_states)
            if self.curr_state[0]==cspace.goal_state[0] and self.curr_state[1]==cspace.goal_state[1]:
                run = False
                self.curr_state[2]=0
                self.curr_state[3]=0
                # print("it will end soon", run)
          
            costs = []
            for s in self.visited_states:    
                control_force = 0.1
                # cost, control_force= cal_cost(s, self.curr_state,accx,accy, accx+accy, obstacle_encountered)
                end_time = time.time()
                print("time", (end_time-start_time)*1000)
                opt_time, cost = self.opt.get_optimal_time(np.asmatrix(s).T,np.asmatrix(self.curr_state).T)
                end_time = time.time()
                print("after time", (end_time-start_time)*1000)
                # print("got cost")
                costs.append(cost)

            min_cost = min(costs)
           
            parent_id = costs.index(min(costs))
            parent = self.visited_states[parent_id]
            self.visited_states.append(self.curr_state)
            # print(self.visited_states)
            self.existing_states[str(self.curr_state)] ={
                                                            'parent':parent,
                                                            'cost_p':control_force,
                                                            'cost_tot':self.existing_states[str(parent)]['cost_tot'] + min_cost}

                ### Rewiring ####
            print(self.existing_states)
            for key, value in self.existing_states.items():
                # print("rewiring")
                # if (end_time-start_time> .020):
                #     run = False
                key_ = self.convert_key(key)
                # accx, accy = get_trajectory(self.curr_state, key_)
                # cost, control_force = cal_cost(self.curr_state, key_, accx, accy, accx+accy, obstacle_encountered)
                opt_time, cost = self.opt.get_optimal_time(np.asmatrix(self.curr_state,).T,np.asmatrix(key_).T)
                if cost == 0.0:
                    cost = .001
                cost_sum = self.existing_states[str(self.curr_state)]['cost_tot']+ cost                                      
                if (cost_sum < value['cost_tot']):
                    self.existing_states[str(key)]['parent'] = self.curr_state
                    self.existing_states[str(key)]['cost_p'] = control_force
                    self.existing_states[str(key)]['cost_tot'] = cost_sum
        # print("planning done")
        path = []
        curr_state = self.curr_state
        cost=[]
        path.append(curr_state)
        cost.append(0.0)
        count = 0
        # while curr_state != cspace.start_state:
        while count <3:
            # print("in the last loop", curr_state, cspace.goal_state, cspace.start_state)
            prev_state = curr_state
            # print("parent",self.existing_states[str(curr_state)]['parent'], curr_state,self.existing_states[str(prev_state)]['cost_p'])
            curr_state = self.existing_states[str(prev_state)]['parent']
            curr_cost =self.existing_states[str(prev_state)]['cost_p']
            path.append(curr_state)
            cost.append(curr_cost)
            count = 1+count

        cost[0] = .05
        # print(end_time-start_time)
        return path, cost
    

if __name__ == '__main__':
    cs = cspace()
    k_rrt =krrt()
    path = k_rrt.krrt_star(cs)
    print(path)