from shapely.geometry import Polygon, Point
from shapely.affinity import translate, scale
import numpy as np
import time
# from shapely.affinity import affine_transform
# Create a polygon


class MyPolygon:

    # input id and coordinates in clockwise 
    def __init__(self, id, polygon_coords):
        self.id = id
        self.polygon = Polygon(polygon_coords)


    # # Check if a point is inside the polygon
    # point = (0.5, 0.5)  # Example point coordinates
    def check_pt_inside_poly(self, point):
        is_inside = self.polygon.contains(Point(point[0], point[1]))
        return is_inside

    def get_poly_info(self):
        area = self.polygon.area
        perimeter = self.polygon.length
        centroid = self.polygon.centroid
        bounding_box = self.polygon.bounds
        return area, perimeter, centroid, bounding_box


    def display(self):
        import matplotlib.pyplot as plt
        x, y = self.polygon.exterior.xy
        plt.plot(x, y)
        plt.show()        


class AgentPolygon():
    # input polygons
    def __init__(self, agent_polygon, polygons):
        self.agent = agent_polygon.polygon
        self.polygons = polygons #self.polygon is a list of Mypolygon 
        self.inflate_polys()

    def inflate_polys(self):
        self.agent_inflated= self.agent.buffer(0.5)
        self.polys_inflated = []
        for poly in self.polygons:
            self.polys_inflated.append(poly.polygon.buffer(0.5))


    def check_poly_collision(self):
        for polygon in self.polys_inflated:
            has_collided = not self.agent_inflated.disjoint(polygon)
        return has_collided


    # should obstacle position also be updated based on some predictions??
    # sensing is done only once is 20 mini seconds
    def step_agent(self, trans_matrix):
        self.agent = translate(self.agent,xoff=trans_matrix[0],yoff=trans_matrix[1])
        self.agent_inflated = translate(self.agent_inflated,xoff=trans_matrix[0],yoff=trans_matrix[1])
        # print("agent moved")

    def get_kenimatics():
        pass

    def get_agent_dynamics():
        pass

    def get_sensor_data():
        pass

    def get_sensor_forcer_reaction():
        pass


    def display(self):
        import matplotlib.pyplot as plt
        x, y = self.agent.exterior.xy
        plt.plot(x, y)
        for poly in self.polygons:
            x, y = poly.polygon.exterior.xy
            plt.plot(x, y)
        plt.show() 


if __name__ == "__main__":

    # Example usage
    polygon_coords1 = [(1, 1), (1, 5), (4, 5), (4, 1)]  # Example coordinates for a polygon
    polygon_coords2 = [(3, 3), (3, 6), (6, 6), (6, 3)]  # Example coordinates for another polygon

    polygon1 = MyPolygon(0,polygon_coords1)
    polygon2 = MyPolygon(1,polygon_coords2)
    # point = (2,3)
    # is_inside = polygon1.check_pt_inside_poly(point)

    agent_env = AgentPolygon(polygon1, [polygon2])
    agent_env.check_poly_collision()
    # polygon1.display()
    start_time =  time.time()
    
    translation_values = np.array([-2, -2])  # x translation, y translation

    agent_env.step_agent(translation_values)
    print("collision check", agent_env.check_poly_collision())
    end = time.time()
    print(end-start_time)
    # Example translation values
  
    agent_env.display()
    


