Robot Path Planning using RRT Algorithm

This code provides a simple implementation of the RRT algorithm for robot path planning. The robot moves on a 2D grid with obstacles and tries to find a path to reach the end point.

Input:
- The code uses a fixed starting point and goal point, which are defined in the `main` function of the `rrt.py` file. You can modify these points to specify different starting and goal locations.
- Obstacles are defined as an array in [(x,y,radius)] format in the `main` function. You can add or remove obstacle points to create different scenarios.

Output:
- Once a path is found from the starting point to the goal point, the code will plot the path.

Errors :
- I am unable to make the path totally obstacle free.

Steps to Compile and Run :

1. Open the terminal and navigate to the directory where you want to clone the repository. Then clone the repository 
2. To run the program go to the terminal and navigate to the directory and run "python rrt.py"
