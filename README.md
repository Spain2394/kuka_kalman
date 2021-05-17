# kuka_kalman
Localize omnidirectional robot in Gazebo 

# Getting Started
[Install](http://www.youbot-store.com/wiki/index.php/Gazebo_simulation) KUKA model robot, and robot drivers. 

# Starting project from launch file
Assumes youbot came from http://github.com/youbot/youbot_simulation.git and http://github.com/youbot/youbot_description.git. 
Make sure these projects are sourced so ROS can find them.
After running catkin_make in workspace, you can use launch file 
to generate an empty world with the YouBot robot initialized to position (0,0,0). It also starts the sensor_model node, kf_node, and the Move_You node. These are the nodes for the sensor model, Kalman filter localization, and teleoperation drivers

-----------------------------------
### Robot Driver
Provided by youbot_driver

### Localization
Publishes the bot position given command velocity and sensor data. Also used to publish the error between localization position and the bot's true position.

### Sensor Driver
Assumes position variables x and y are independent of each other, and that their covariance is 0 between them. Applies Gaussian noise to the ground truth position. Assumes a standard deviation of 0.2 for the Gaussian noise for both x and y.

### Teleop Driver
Receives inputs from keyboard and converts them into command velocities which drive the bot at 1 m/s forwards (N), backwards (S), left(E) and right(W). 

![1](https://github.com/Spain2394/Extended-Kalman-ROS/blob/master/src/kuka_kickass_kalman/ScreenCaps/simulation.gif)

## To run youbot and teleop (Move_You)
1) run ```$ catkin_make install``` inside of the ```catkin_ws``` workspace
2) Inside your ```catkin_ws``` run ```$ source devel/setup.bash```
3) run ```$ roscore```
4) in a new terminal run: ```$ roslaunch youbot_gazebo_robot youbot_base_only.launch``` 
5) Wait for Gazebo to build world and stage bot
6) In another terminal run ```$ rosrun kuka_kickass_kalman Move_You.py``` 
7) You can now type commands, either N, S, E, or W to move the youbot 1 meter in the prescribed direction

## To start all project nodes: Move_You, kf_node, and sensor_model
run ```$ roslaunch kuka_kickass_kalman youbot_kalman.launch```

## Custom Messages
To observe localization odom run ```rostopic echo geometry_msgs/Pos/xEst```
To observe error in odom run ```rostopic echo geometry_msgs/Pos/error```

## Plots
to plot odometry uncomment ```kf.graph()``` in ```kf_node.py```

The blue line is true trajectory, the black line is dead reckoning trajectory,

the green point is positioning observation

The red circle is estimated covariance in EKF


## Contributions
**Allen Spain**
Construct this README. 
Constructed group project in github. 
Constructed function outline. 
Created localization script. 
Odometry topic testing and screen captures.
Data plots

**Benjamin Ware** 
Publication node. 
Project sync. 
Kalman subscription node. 
Localization node. 

**Joshua Shannon** 
Sensor node with custom msg. 
Launch file construction. 
Testing/debugging. 

**Stephen Whitcomb** 
User input/ Telop node. 
Program testing. 
Screen captures. 

**Shan Qamruddin** 

## References
* Extended Kalman Localization reference: [github.com/AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics/tree/master/Localization/extended_kalman_filter)

## Reflections



