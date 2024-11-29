
# assignment1_rt

#research track assignment

Turtlesim Safety Control with ROS 
Overview
This project implements two ROS nodes within the package assignment1_rt to control and monitor two turtles in the turtlesim environment. The nodes enforce safety rules that prevent turtles from colliding or moving out of bounds:

UI Node (node1): Allows the user to control either turtle1 or turtle2 by specifying velocity commands. After each input, the turtles will move for 1 second and then stop. The user can then input a new command.

Distance Node (node2): Continuously checks the relative distance between turtle1 and turtle2. It:

Publishes the distance between the turtles.
Stops the turtles if they get too close to each other.
Stops the turtles if they move too close to the boundaries of the environment.
Features
UI Node:

Spawns turtle2 in addition to the default turtle1.
Allows the user to control the turtles by specifying linear and angular velocities.
Validates the input to ensure that movement does not violate safety rules (e.g., turtles becoming too close or moving out of bounds).
Stops the turtle after 1 second of movement and waits for the next input.
Distance Node:

Continuously monitors the distance between turtle1 and turtle2.
Publishes the distance between the turtles on the /turtles_distance topic.
Stops a turtle's movement if the turtles get too close (threshold of 2.0 units).
Ensures that turtles stay within the boundaries of the environment (1.0 ≤ x, y ≤ 10.0).
Installation
1. Install Prerequisites
Ensure you have the following installed:

Ubuntu 20.04 or later
ROS Noetic
2. Setup ROS Workspace
Open a terminal and create a new workspace:
bash
Copy code
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
Source the workspace:
bash
Copy code
source devel/setup.bash
3. Clone and Build the Project
Navigate to the src directory:
bash
Copy code
cd ~/catkin_ws/src
Clone the repository (replace <repo-url> with the actual link):
bash
Copy code
git clone <repo-url>
Build the workspace:
bash
Copy code
cd ~/catkin_ws
catkin_make
4. Source the Workspace
Make sure to source the workspace so ROS can find the new package:

bash
Copy code
source ~/catkin_ws/devel/setup.bash
Running the Nodes
1. Launch the Turtlesim Simulator
Open a terminal and start the turtlesim node:

bash
Copy code
rosrun turtlesim turtlesim_node
2. Run the Distance Node (node2)
In a new terminal, run the Distance Node:

bash
Copy code
rosrun assignment1_rt distance_node.py
This node will begin monitoring the distance between turtle1 and turtle2 and apply the safety rules.

3. Run the UI Node (node1)
In another new terminal, run the UI Node:

bash
Copy code
rosrun assignment1_rt ui_node.py
This node will allow you to interactively control the turtles by entering velocity commands for turtle1 or turtle2.

How to Use
UI Node (node1)
Select the Turtle:
The program will prompt you to select either turtle1 or turtle2:

Enter 1 to control turtle1.
Enter 2 to control turtle2.
Enter Velocities:

You will be asked to provide a linear velocity (forward/backward speed).
You will also be asked to provide an angular velocity (turning speed).
Movement Behavior:

The turtle will move for 1 second and then stop.
The program will allow the user to enter a new command after each stop.
The system will block movement if the turtles get too close to each other (below 2.0 units).
Movement will also be blocked if the turtles try to move outside the environment boundaries (1.0 ≤ x, y ≤ 10.0).
Distance Node (node2)
The Distance Node continuously checks and publishes the distance between turtle1 and turtle2 on the topic /turtles_distance.
If the turtles get too close, the distance node will send stop commands to the turtles to prevent collisions.
Conditions for Movement
Safe Distance:
The turtles cannot move closer than 2.0 units to each other.

Commands that increase the distance are allowed.
Commands that reduce the distance are blocked.
Boundary Restriction:
The turtles must stay within the boundaries:

x and y values must be between 1.0 and 10.0.
Commands that move the turtles out of bounds are ignored.
Running Code in Different Terminals
To run the nodes, you will need to open three different terminals. Follow these steps:

Terminal 1 - Turtlesim Node:
Start the turtlesim environment:

bash
Copy code
rosrun turtlesim turtlesim_node
Terminal 2 - Distance Node:
Run the Distance Node (node2):

bash
Copy code
rosrun assignment1_rt distance_node.py
Terminal 3 - UI Node:
Run the UI Node (node1):

bash
Copy code
rosrun assignment1_rt ui_node.py

