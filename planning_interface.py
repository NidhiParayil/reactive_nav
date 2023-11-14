from scripts import pybullet_env
import numpy as np
import time

from planner import rrt_star, krrt_star, ompl_planners
import cspace
import pybullet as p




maze = pybullet_env.MazeEnv()
cs = cspace.StateSampler()
maze.debug_points([cs.goal_state[0], cs.goal_state[1],1])
obs_polygon_edges =  maze.obs_state()



# planner =krrt_star.krrt() #krrt*


planner = rrt_star.RRT_star() #rrt*


# planner = ompl_planners.OMPL_planner(cs) # ompl

start_time = time.time()


# path, force_cost = planner.run_planner(cs,[[0.,0.,0.], [.1,.1,0.]]) #krrt*

path, force_cost = planner.run_planner(cs) #rrt*


# path, force_cost = planner.run_planner(cs.start_state, cs.goal_state) # ompl

print("goal is",cs.goal_state )
end_time = time.time()
print('time taken for krrt path in milli seconds:', (end_time - start_time)*1000)
# path.reverse()
# force_cost.reverse()
print("PATHHHHHHHHH")
print(path)
print("Force")
print(force_cost)

count =1

points=[]
colour =[]
rob_points =[]
obs_points = []
input("Press enter to continue")



while(count < len(path)):
    points.append([path[count][0],path[count][1],6])
    colour.append([1, count/len(path),0])

# ******
# have to write a non bouncing controller

# *******
    dis = 100
    while( (dis>.2) or (dis >.01 and count ==len(path)-1)) :
        cur_x, cur_y, obs= maze.step(path[count][0],path[count][1], np.abs(force_cost[count]))
        dis = np.sqrt((cur_x-path[count][0])**2 +(cur_y-path[count][1])**2 )
        rob_points.append([cur_x,cur_y,6.2])
        obs_points.append([2,2.5+obs,6.3])
        # print(dis, count)
    count = count+1
    

maze.debug_traj(points, [1,0,0],lineW=3)
maze.debug(rob_points, [1,1,0],lineW=3)
# maze.debug(obs_points, [0,0,1],lineW=3)

print("all done")
time.sleep(10)


