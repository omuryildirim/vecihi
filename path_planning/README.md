Algorithm to allow a robot to cover whole field that contains pre-defined obstacles. Algorithm based on vertical cell decomposition.

Vertical Cell Decomposition Result
![Vertical Cell Decomposition](https://gitlab.ldv.ei.tum.de/ga56tew/AutoDrones/raw/master/cognition/path_planning/prints/input_file_6/map_with_image.png)

### Files

***

#####helpers:
The helpers folder contains four files 
    - decomposition: cell decomposition functions
    - euler_tour: Algorithm to calculate Euler tour from Reeb Graph
    - geometry: contains functions line finding segment intersection
    - graph: main reeb graph creator function
    - map_tools: plot functions
    
#####maps:
Maps folder contains files which contains map details
    - Map file format are as follows: 
        * Starting line is bounds of map
        * Each line contains end points for a polygon describing an obstacle. The end points are arranged in a counter-clockwise direction.

#####prints:
Prints folder contains result for each map

_________________________________________


drone.py: Classes for Drone and Obstacle. All motion calculation done under Drone class. Also stores poly data for Drone and Obstacles.
drone_3d.obj: 3D vector for rendering drone in simulator.
sensor.py: LIDAR sensor class.
simulator.py: Run this file to start simulator. You can call simulator from terminal by calling `simulator director.py` command
vertical_cell_decomposition.py: Class for vertical cell decomposition.
world.py: Simulator world generation class.

