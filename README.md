# GP 2020 Simulation
## A ROS simulation package for our self-driving car graduation project
 
 - **Step 1:**  Set up ROS Melodic (Desktop Full)
        http://wiki.ros.org/melodic/Installation/Ubuntu

 - **Step 2:**  Install required packages by cloning them in your catkin_ws/src directory

       git clone https://github.com/ros-drivers/openni_camera.git
       git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
       git clone https://github.com/ros-drivers/freenect_stack.git
       git clone https://github.com/ros-drivers/libfreenect.git
       git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
       git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
       git clone https://github.com/ros-drivers/ackermann_msgs.git

 - **Step 3:**  Run the project by doing the following:

1. Run the main controller and simulation
```properties

foo@bar:~$ roscd
foo@bar:~$ cd ../src
foo@bar:~$ roslaunch gp2020_simulation gp2020_simulaion.launch

```
2. Run the obstacle detection package

```properties

foo@bar:~$ roscd
foo@bar:~$ cd ../src
foo@bar:~$ roslaunch object_detection object_detection.launch

```
3. Run the lane detection package

```properties

foo@bar:~$ roscd
foo@bar:~$ cd ../src
foo@bar:~$ roslaunch lane_detection lane_detection.launch

```

