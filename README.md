# arduinobot
To run the simulation in Gazebo you need to have ignition gazebo 6 installed:

Note : humble distro

Following CLI help you out in running the simulation:

- ros2 launch arduinobot_description gazebo.launch.py
- ros2 launch arduinobot_controller controller.launch.py
- ros2 launch arduinobot_moveit moveit.launch.py

You can implement scripts in 2 ways :
- python (works in jazzy)
- c++ (works in humble)

Here I implemeted for c++:
- ros2 run arduinobot_moveit simple_moveit_interface

Working:
Here the code takes input from the user in the form of number between 1-9 
each number respresent various pose of actions for every input in num hit enter
