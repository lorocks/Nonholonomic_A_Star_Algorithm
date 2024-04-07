# ENPM661_Project3_Phase2
ROS2 Packages to simulate a TurtleBot3 after optimal path detection using A* algorithm.

The following implementation uses A* algorithm to find the optimal path to the goal given a restriced action set.

## GitHub Link
https://github.com/lorocks/Nonholonomic_A_Star_Algorithm/

## Installing Dependencies
```bash
# Install numpy
  pip install numpy
# Install opencv
  pip install opencv-python
```

## Action Set
The action set consists to 8 actions.
Actions - [
    (0, rpm1),
    (rpm1, 0),
    (rpm1, rpm1),
    (0, rpm2),
    (rpm2, 0),
    (rpm2, rpm2),
    (rpm1, rpm2),
    (rpm2, rpm1)
]


# Part - 1
In this part, A* algorithm is used to search the optimal path using the action set.

The code will either output a final path shown as an image or create a video of the A* algorithm exploration.

## Code Files
There are 2 code files and both use different methods to generate obsatcles.

1. nonholonomic_a_star.py - Finds optimal path using A* algorithm, and cv2 for obstacle map generation
2. nonholonomic_a_star_half_plane_grid.py - Finds optimal path using A* algorithm, and half planes and semi-algebraic equations for obstacle map generation

## User Inputs
The program will prompt you to add the clearance value

Next, the RPM values needs to be input one by one

The program will prompt you to add the starting location as, (x, y, ϴ)

x - starting x location

y - starting y location

ϴ - starting yaw angle orientation 


Next, the goal location needs to be input, (x, y)

x - goal x location

y - goal y location


## Code Execution
The code can be executed in any way.

After code execution takes place, a video showing the search will pop up.

## Project Video
The following video shows the grid search for starting location (300, 1700, 0) and goal location (5750, 300)




# Part 2
In this part, the optimal path generated is input as actions using a open controller for Turtlebot3 in Gazebo.

The ROS files are found inside the /src directory.

The preferred method of execution is given below.

## Building
```bash
# To build the project run,
# Move to the ros2 workspace
  colcon build
```

## Launching
### Launch Gazebo (only)
```bash
# To launch the car only in Gazebo,
# Move to the ros2 workspace and source the underlay
  source install/setup.sh
# Launch Gazebo with car
  ros2 launch turtlebot3_project3 competition_world.launch.py
```
#### If Failure
If the Gazebo launch fails,
<br>
End the session and repeat the above steps for launching

## Run Path Generator and Open Controller
### Run Node
The default start, goal and RPM values are (500, 1000, 0), (5750, 1000), and 10 and 20.

More parameters can be found inside auto_controller.py

```bash
# Move to the ros2 workspace and source the underlay
  source install/setup.sh
# Run the script
  ros2 run turtlebot3_project3 auto_controller.py
# Run the script with different goal point
  ros2 run turtlebot3_project3 auto_controller.py --ros-args -p goal_x:=< Enter goal x > -p goal_y:=< Enter goal y >
```

## Preferred Method of Execution
The preferred method of execution to minimise execution time is given below.

1. Clone the GitHub
2. Build the Project
3. Source the Project
4. Run the Open Controller Node
5. Wait for Final Path Image to be Generated
6. Launch Gazebo World
7. Close Image to Start the Execution