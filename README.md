## RRT* Path Planning algorithm in 3D

Finds path from start to goal by sampling random nodes within the discretized environment and builds a 
tree structure from the start to goal.

Will save 2 images: map.png which shows the environment, and then waypoints.png which plots the waypoints onto the environment

Able to create your own world as a json package. Json must contain key words:
 - bounds
 - blocks
 - start
 - goal
 - margin
 - resolution

Usage:
```
git clone git@github.com:anthonyn2121/rrt_star.git
git submodule update --init --recursive
python3 rrt.py -f <json_file>
```
Example:
```
python3 rrt.py -f environment_toolkit/worlds/forest.json
```
