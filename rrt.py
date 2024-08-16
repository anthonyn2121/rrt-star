import setup
import json
import numpy as np
from environment_toolkit.environment import Environment
from environment_toolkit.occupancy_map import OccupancyMap
from node import Node

# https://arxiv.org/pdf/1105.1186
# https://roboticsproceedings.org/rss06/p34.pdf
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

    def plan(self, start, goal, max_iters=100, goal_radius=3.0):
        ## Start a vertex set with only the initial point and and empty edge set
        self.nodes = [Node(start[0], start[1], start[2])]
        goal_node = Node(goal[0], goal[1], goal[2])
        best_goal_node = None
        for _ in range(max_iters):
            ## Randomly select a point in free space
            sampled_node = self.__random_sample()
            ## Find nearest vertex
            nearest_node = self.__get_closest_node(sampled_node)
            ## Find the a point on the line from the nearest_node to the sampled_node within a step size
            new_node = self.__steer(nearest_node, sampled_node, step_size=2.0)

            if (self.__is_collision_free(nearest_node, new_node)):
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + distance(nearest_node, new_node)

                ## Find nearest nodes
                RADIUS = 3.0
                nearest_nodes = [node for node in self.nodes if distance(node, new_node) < RADIUS]

                ## Rewire nodes
                for node in nearest_nodes:
                    new_cost = new_node.cost + distance(new_node, node)
                    print("self.__is_collision_free(node, new_node) and (new_cost < node.cost):")
                    print(self.__is_collision_free(node, new_node) and (new_cost < node.cost))
                    if (self.__is_collision_free(node, new_node) and (new_cost < node.cost)):
                        node.parent = new_node
                        node.cost = new_cost
                        print(node)

                ## Check if connection to goal is valid
                if distance(new_node, goal_node) < goal_radius and\
                   self.__is_collision_free(new_node, goal_node):
                    potential_goal_node = Node(goal[0], goal[1], goal[2],
                                               parent = new_node,
                                               cost = new_node.cost + distance(new_node, goal_node))

                    if (best_goal_node is None) or (potential_goal_node.cost < best_goal_node.cost):
                        best_goal_node = potential_goal_node

        if (best_goal_node):
            return self.__trace_path(best_goal_node)
        else:
            return None
        
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
            if self.occupancy.is_valid_position(node.to_array()):
                valid_node = True

        return node

    def __get_closest_node(self, sample_node:Node) -> Node:
        '''
        Get the closest node in the tree
        '''
        cost_map = {node:distance(node, sample_node) for node in self.nodes}
        return min(cost_map, key=cost_map.get)
    
    def __steer(self, nearest_node:Node, sampled_node:Node, step_size=1.0) -> Node:
        '''
        steer the nearest node towards the sampled node given the step size
        '''
        
        # d = abs(distance(nearest_node, sampled_node))
        if (step_size > distance(nearest_node, sampled_node)):
            return AssertionError("Step size of __steer needs to be less than the distance between points")
        v = abs(nearest_node - sampled_node) ## difference between vectors
        return nearest_node + (step_size * Node.normalize(v))

    def __is_collision_free(self, from_node, to_node) -> bool:
        d = distance(from_node, to_node)
        step_sizes = np.linspace(0, d, 50)
        for s in step_sizes:
            point = self.__steer(from_node, to_node, step_size=s)
            if (self.occupancy.is_occupied_position(point.to_array())):
                return False
        return True

    def __trace_path(self, best_goal_node:Node) -> np.array:
        path = []
        current_node = best_goal_node
        while current_node:
            path.append(current_node.to_array())
            current_node = current_node.parent
        return path[::-1]
    
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', default="environment_toolkit/worlds/test_simple.json", help="json file that describes an environment")
    args = parser.parse_args()


    world_filepath = args.file
    with open(world_filepath, 'r') as file:
        world = json.load(file)

    world_data = {key:world[key] for key in ["bounds", "blocks"]}
    env = Environment(world_data)
    start = tuple(world['start'])
    goal = tuple(world['goal'])
    margin = world['margin']
    resolution = world['resolution']

    rrt = RRT(env, resolution)
    print(rrt.plan(start, goal, max_iters=1))
