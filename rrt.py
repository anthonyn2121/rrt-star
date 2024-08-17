import setup
import json
import numpy as np
import matplotlib.pyplot as plt
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

def get_path_cost(node:Node):
    sum = node.cost
    curr_node = node.parent
    # print("node: ", node, "parent: ", node.parent)
    while (curr_node):
        # print(f"sum={sum}, curr_node={curr_node}")
        sum += curr_node.cost
        curr_node = curr_node.parent
    return sum

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
        potential_best_node = None
        best_goal_node = None
        for _ in range(max_iters):
            ## Randomly select a point in free space
            sampled_node = self.__random_sample()
            ## Find nearest vertex
            nearest_node = self.__get_closest_node(sampled_node)
            ## Find the a point on the line from the nearest_node to the sampled_node within a step size
            new_node = self.__steer(nearest_node, sampled_node, step_size=1.0)
            if (self.__is_collision_free(nearest_node, new_node)):
                ## Find nearest nodes
                RADIUS = 1.0
                nearest_nodes = {node:(get_path_cost(node) + distance(node, new_node)) 
                                    for node in self.nodes if (distance(node, new_node) < RADIUS)}
                if (len(nearest_nodes) <= 0) : continue
                nearest_node, min_cost = min(nearest_nodes.items(), key=lambda x: x[1])
                new_node.parent = nearest_node
                new_node.cost = min_cost
                self.__rewire(nearest_nodes, new_node)
                self.nodes.append(new_node)

            if ((distance(new_node, goal_node) < goal_radius) and 
                    self.__is_collision_free(new_node, goal_node)):
                potential_best_node = Node(goal_node.x, goal_node.y, goal_node.z,
                                           parent=new_node,
                                           cost=new_node.cost + distance(new_node, goal_node))
        
            if ((best_goal_node is None) or 
                    (potential_best_node.cost < best_goal_node.cost)):
                best_goal_node = potential_best_node
        
        if (best_goal_node):
            node_path = self.__trace_path(best_goal_node)
            return np.asarray([node.to_array() for node in node_path])
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
            if ((self.occupancy.is_valid_position(node.to_array())) and 
                (not self.occupancy.is_occupied_position(node.to_array())) and
                (node not in self.nodes)):
                valid_node = True

        return node

    def __get_closest_node(self, sample_node:Node) -> Node:
        '''
        Get the closest node in the tree
        '''
        cost_map = {node:distance(node, sample_node) for node in self.nodes}
        return min(cost_map, key=cost_map.get)
    
    def __steer(self, from_node:Node, to_node:Node, step_size=1.0) -> Node:
        '''
        steer the nearest node towards the sampled node given the step size
        '''
        
        if (step_size > distance(from_node, to_node)):
            return to_node
        v = abs(from_node - to_node) ## difference between vectors
        return from_node + (step_size * Node.normalize(v))

    def __is_collision_free(self, from_node, to_node) -> bool:
        d = distance(from_node, to_node)
        step_size = d/74
        while not (step_size >= d):
            point = self.__steer(from_node, to_node, step_size)
            if (not self.occupancy.is_valid_position(point.to_array()) or
                self.occupancy.is_occupied_position(point.to_array())):
                return False
            step_size += step_size
        return True

    def __rewire(self, nearest_neighbors:dict, new_node:Node):
        for node, cost in nearest_neighbors.items():
            new_cost = new_node.cost + distance(node, new_node)
            if (new_node == node): continue
            if (new_cost < cost and self.__is_collision_free(new_node, node)):
                node.parent = new_node
                node.cost = new_cost

    def __trace_path(self, node:Node) -> np.array:
        path = []
        current_node = node
        while current_node:
            path.append(current_node)
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
    waypoints = None
    while (not np.any(waypoints)):
        waypoints = rrt.plan(start, goal, max_iters=2000, goal_radius=1.0)
    xs, ys, zs = zip(*waypoints)

    ax = env.get_plot()

    ax.plot(xs, ys, zs, color='b', marker='o')
    ax.plot([start[0]], [start[1]], [start[2]], color='green', marker='o')
    ax.text(start[0], start[1], start[2], "START", color='green')
    ax.plot([goal[0]], [goal[1]], [goal[2]], color='red', marker='o')
    ax.text(goal[0], goal[1], goal[2], "GOAL", color='red')
    plt.show(block=True)