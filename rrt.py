import setup
import json
import numpy as np
from environment_toolkit.environment import Environment
from environment_toolkit.occupancy_map import OccupancyMap

# https://arxiv.org/pdf/1105.1186
# https://roboticsproceedings.org/rss06/p34.pdf
class Node:
    def __init__(self, x, y, z, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent

    def copy(self):
        return Node(self.x, self.y, self.z, self.parent)

    def to_array(self):
        return np.array([self.x, self.y, self.z])

    def __add__(self, rhs):
        return Node(self.x + rhs.x, self.y + rhs.y, self.z + rhs.y)
    
    def __sub__(self, rhs):
        return Node(self.x - rhs.x, self.y - rhs.y, self.z - rhs.y)
    
    def __mul__(self, rhs):
        # scalar multiplication
        return Node(rhs*self.x, rhs*self.y, rhs*self.z)
    
    @staticmethod
    def normalize(node):
        x, y, z = node.x, node.y, node.z
        magnitude = np.sqrt(x**2 + y**2 + z**2)
        return Node(x/magnitude, y/magnitude, z/magnitude)

def distance(from_node, to_node):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    dz = to_node.z - from_node.z
    return np.sqrt(dx**2 + dy**2 + dz**2)

class RRT:
    def __init__(self, world:Environment, resolution):
        self.world = world
        self.occupancy = OccupancyMap(world, resolution)
        self.resolution = resolution
        self.nodes = []
        self.edges = set()

    def plan(self, start, goal, max_iters=100):
        ## Start a vertex set with only the initial point and and empty edge set
        self.nodes = [Node(start[0], start[1], start[2])]
        for _ in range(max_iters):
            ## Randomly select a point in free space
            sampled_node = self.__random_sample()
            ## Find nearest vertex
            nearest_node = self.__get_closest_node(sampled_node)
            ## Find the a point on the line from the nearest_node to the sampled_node within a step size
            new_node = self.__steer(nearest_node, sampled_node, step_size=1.0)

            if (self.__is_collision_free(nearest_node, new_node)):
                ## Find nearest nodes
                pass
                
                # RADIUS = 3.0
                # cost_map = {node: cost(new_node, node) for node in self.nodes}
                # nearest = dict(filter(lambda item: item[1] < RADIUS, cost_map.items()))  ## Get key-value pairs where value < RADIUS
            

    def __random_sample(self) -> Node:
        x_min, x_max, y_min, y_max, z_min, z_max = self.world.map_bounds

        node = np.zeros(3)
        valid_node = False
        while not valid_node:
            # Random sample
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)
            z = np.random.uniform(z_min, z_max)

            # Scale towards resolution
            x = self.resolution * round(x/self.resolution)
            y = self.resolution * round(y/self.resolution)
            z = self.resolution * round(z/self.resolution)

            node = Node(x, y, z)
            if self.occupancy.is_valid_position(node):
                valid_node = True

        return node

    def __get_closest_node(self, sample_node:Node) -> Node:
        '''
        Get the closest node in the tree
        '''
        cost_map = {node:distance(node, sample_node) for node in self.node}
        return min(cost_map, key=cost_map.get())
    
    def __steer(self, nearest_node:Node, sampled_node:Node, step_size=1.0) -> Node:
        '''
        steer the nearest node towards the sampled node given the step size
        '''
        v = nearest_node - sampled_node ## difference between vectors
        return nearest_node + (step_size * Node.normalize(v))

    def __is_collision_free(self, from_node, to_node) -> bool:
        d = distance(from_node, to_node)
        step_sizes = np.linspace(0, d, 50)
        for s in step_sizes:
            point = self.__steer(from_node, to_node, step_size=s)
            if (self.occupancy.is_occupied_position(point.to_array())):
                return False
        return True


if __name__ == "__main__":
    import argparse
   
    world_data = '''{
        "bounds": [0, 10, 0, 10, 0, 10],
        "blocks": [
            {"position":[1, 1, 1], "size": [2, 2, 2]},
            {"position":[5, 5, 0], "size": [3, 3, 3]},
            {"position":[7, 2, 4], "size": [1, 4, 1]}
            ],
        "start": [4, 2, 0],
        "goal": [9, 8, 8],
        "margin": 0.5,
        "resolution": 0.5
        }'''

    world = json.loads(world_data)
    world_data = {key:world[key] for key in ["bounds", "blocks"]}

