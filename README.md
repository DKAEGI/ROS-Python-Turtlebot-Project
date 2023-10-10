# ROS-Python-Turtlebot-Project

A ROS2 Project to implement the Basics of ROS2 in C++. <br>
Integration of a Custom Service Message. <br>
Integration of Service Server to find the nearest wall. <br>
Integration of a Service Client to call the Service Server. <br>
Integration of a Custom Action Message. <br>
Integration of a Action Server to record the Odometry of the robot. <br>
Integration of a Action Client to start the recording of the Odometry. <br>
A script which makes the robot to follow the wall on the right side for a specific distance. <br>
A handle error case, when the the robot collidess with the wall, such that it can restart the movement. <br>

The Script has been successfully tested on the Turtlebot. <br>
The Project was done in ROS Noetic. <br>

Possible improvements: <br>
Use a range of laser beams instead of just one laser_beam value. <br>
Instead of just doing a front-loop for velocity control, use a feedback loop with some kind of PID Control <br>
