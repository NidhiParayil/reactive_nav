
import sys 
from os.path import abspath, dirname, join
sys.path.insert(0, '/home/nidhi/ompl/py-bindings')
sys.path.append('/home/nidhi/pybullet_ws/planner/')
sys.path.append('/home/nidhi/pybullet_ws/')
from ompl import base, geometric
from planners import Planner
from scripts import polygon
from math import sqrt
import numpy as np
import math

# *****
# implementing polygon collision check with SAT algorithm
# *****

class ValidityChecker(base.StateValidityChecker):
     # Returns whether the given state's position overlaps the
     # circular obstacle
    def isValid(self, state, ob_sense_states =  [(0.5, 1.0), (1., 1.0), (1., 1.5), (0.5, 1.5)]):
        obstacle = polygon.MyPolygon(1,ob_sense_states)
        agent_states = [(state[0]-.5, state[1]-.5), (state[0]+.5, state[1]-.5), (state[0]+.5, state[1]+.5), (state[0]-.5, state[1]+.5)]
        # print("enter collision check", state[0], state[1]) 
        agent = polygon.MyPolygon(0,agent_states)
        agent_env = polygon.AgentPolygon(agent, [obstacle])
        
        # print(self.clearance(state) > 0.0, not(agent_env.check_poly_collision()))
        # agent_env.display()
        return agent_env.check_poly_collision()
  
     # Returns the distance from the given state's position to the
     # boundary of the circular obstacle.
    def clearance(self, state):
        # Extract the robot's (x,y) position from its state
        x = state[0]
        y = state[1]
  
         # Distance formula between two points, offset by the circle's
         # radius
        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25
  

class CustomOptimizationObjective(base.OptimizationObjective):
    def __init__(self, si):
        super(CustomOptimizationObjective, self).__init__(si)
    
    # Define a custom motion cost
    def motionCost(self, a, b, ob_sense_states = [(0.5, 1.0), (1., 1.0), (1., 1.5), (0.5, 1.5)]):
        state = a
        obstacle = polygon.MyPolygon(1,ob_sense_states)
        agent_states = [(state[0]-.5, state[1]-.5), (state[0]+.5, state[1]-.5), (state[0]+.5, state[1]+.5), (state[0]-.5, state[1]+.5)] 
        agent = polygon.MyPolygon(0,agent_states)
        agent_env = polygon.AgentPolygon(agent, [obstacle]) 
         
        from shapely.geometry import LineString

        a_values = [a[0], a[1]]
        b_values = [b[0], b[1]]
        path_line = LineString([(a[0], a[1]), (b[0], b[1])])
        intersection = obstacle.polygon.intersection(path_line)

        if intersection.is_empty:
            length = 0
        elif intersection.geom_type == 'LineString':
            length = intersection.length
        else:
            length = sum(segment.length for segment in intersection)

        # if agent_env.check_poly_collision() == False:
            # print("collided")

        return np.linalg.norm(np.array(a_values) - np.array(b_values)) 

    # Define a custom cost to go heuristic
    def costToGo(self, s):
        return self.motionCost(s, self.getGoal())

    def test(self, si):
        return base.PathLengthOptimizationObjective(si)


class OMPL_planner(Planner):

    def __init__(self, state_sampler):
        print("initializing ompl planner")
        self.plannerType = 'RRTstar'
        self.objectiveType = 'PathLength'
        print("using planner and objecive as", self.plannerType, self.objectiveType)
        self.statesampler = state_sampler
        self.start = base.State(self.statesampler.space)
        self.goal = base.State(self.statesampler.space)
        self.si = base.SpaceInformation(self.statesampler.space)
        self.validityChecker = ValidityChecker(self.si)
        self.si.setStateValidityChecker(self.validityChecker)
        self.si.setup()
        self.pdef = base.ProblemDefinition(self.si)
        self.start()[0] = 0.0
        self.start()[1] = 0.0
    
        self.goal()[0] = 1.0
        self.goal()[1] = 3.0
        
        # self.pdef.setOptimizationObjective(self.allocateObjective(self.si, self.objectiveType))
        self.obj = CustomOptimizationObjective(self.statesampler.ss.getSpaceInformation())
        self.pdef.setOptimizationObjective(self.obj)
        self.optimizingPlanner = self.allocatePlanner(self.si, self.plannerType)
        self.optimizingPlanner.setProblemDefinition(self.pdef)
        self.optimizingPlanner.setup()

        


    def getPathLengthObjective(self, si):
        return base.PathLengthOptimizationObjective(si)

    def distance(self, state1, state2):
        distance = 0.0
        for i in range(2):
            distance += (state1[i] - state2[i]) ** 2
        return math.sqrt(distance)


    def allocatePlanner(self, si, plannerType):
        if plannerType.lower() == "bfmtstar":
            return geometric.BFMT(si)
        elif plannerType.lower() == "bitstar":
            return geometric.BITstar(si)
        elif plannerType.lower() == "fmtstar":
            return geometric.FMT(si)
        elif plannerType.lower() == "informedrrtstar":
            return geometric.InformedRRTstar(si)
        elif plannerType.lower() == "prmstar":
            return geometric.PRMstar(si)
        elif plannerType.lower() == "rrtstar":
            return geometric.RRTstar(si)
        elif plannerType.lower() == "sorrtstar":
            return geometric.SORRTstar(si)
        else:
            ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")
  
    def allocateObjective(self, si, objectiveType):
        if objectiveType.lower() == "pathclearance":
            return getClearanceObjective(si)
        elif objectiveType.lower() == "pathlength":
            return self.getPathLengthObjective(si)
        elif objectiveType.lower() == "thresholdpathlength":
            return getThresholdPathLengthObj(si)
        elif objectiveType.lower() == "weightedlengthandclearancecombo":
            return getBalancedObjective1(si)
        else:
            ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")
  
  

    def run_planner(self, start, goal):
        
        

        self.start()[0] = start[0]
        self.start()[1] = start[1]
    
        self.goal()[0] = goal[0]
        self.goal()[1] = goal[1]

        
        self.pdef.setStartAndGoalStates(self.start, self.goal)

        
        solved= self.optimizingPlanner.solve(1)

        if solved:
                # Output the length of the path found
            print('{0} found solution of path length {1:.4f} with an optimization ' \
                'objective value of {2:.4f}'.format( \
                self.optimizingPlanner.getName(), \
                self.pdef.getSolutionPath().length(), \
                self.pdef.getSolutionPath().cost(self.pdef.getOptimizationObjective()).value()))
        
                # If a filename was specified, output the path as a matrix to
                # that file for visualization
            fname = 'test'
            if fname:
                with open(fname, 'w') as outFile:
                    outFile.write(self.pdef.getSolutionPath().printAsMatrix())

            path =self.pdef.getSolutionPath()
            path_length = path.getStateCount()
            state_dimension = self.si.getStateSpace().getDimension()
            path_array = []
            force_array = []
            for i in range(path_length):
                state = path.getState(i)
                state_values = [state[j] for j in range(state_dimension)]
                path_array.append(state_values)
                force_array.append(1)
        else:
            print("No solution found.")
        return path_array, force_array