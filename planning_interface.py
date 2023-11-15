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
path =[(0, 0), (0.6667548633853745, -0.21316179805948013), (1.3333762258673927, 0.0004171313381022834), (1.5402560850059375, 0.669147812466773), (1.5667850293639152, 2.446218362722905), (4.75, 4.75)]
print(path)
print("Force")
print(force_cost)

count =0

points=[]
colour =[]
rob_points =[]
obs_points = []
input("Press enter to continue")


dist_to_goal = 100
dist_to_next_goal = 100
dum_timmer = 0
# points.append([path[0][0],path[0][1],6])

for pt in path:
    points.append(np.asarray([pt[0],pt[1], 6]))
    # print(pt)

maze.debug_traj(points, [1,0,0],lineW=3)



while(dum_timmer <100000 ):

    # print(path[count+1], dist_to_goal,dist_to_next_goal)
    cur_x, cur_y, obs= maze.step(path[count+1][0],path[count+1][1])
    # points.append([path[count][0],path[count][1],6])
    colour.append([1, count/len(path),0])
    dist_to_goal = np.linalg.norm(np.asarray([cur_x,cur_y]) - path[-1])
    dist_to_next_goal =np.linalg.norm(np.asarray([cur_x,cur_y]) - path[count+1])
    if dist_to_goal < .1:
        break
    if dist_to_next_goal < .25:
        count = count+1

        if count == len(path)-1:
            break
    rob_points.append([cur_x,cur_y,6.2])
    obs_points.append([2,2.5+obs,6.3])
    dum_timmer = dum_timmer +1
    


maze.debug_traj(rob_points, [1,1,0],lineW=3)
# maze.debug(obs_points, [0,0,1],lineW=3)

print("all done")
time.sleep(10)


