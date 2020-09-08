## **Gazebo ROS  Where AM I and Ball Chaser**

Link of vídeo https://www.youtube.com/watch?v=eQeoTbMQ3ME

![image-20200908133548258](C:\Users\naube\AppData\Roaming\Typora\typora-user-images\image-20200908133548258.png)



The world map nauber5.world is used in this task 





## File  AMCL.launch

`<launch>`
`<!-- TODO: Add nodes here -->`
`<arg name="map_file" default="/home/workspace/catkin_ws/src/my_robot/maps/nauber5.yaml"/>`
`<node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />`




`<node name="amcl" pkg="amcl" type="amcl" output="screen">`

  `<param name="odom_frame_id" value="odom"/>`
  `<param name="odom_model_type" value="diff-corrected"/>`
  `<param name="base_frame_id" value="robot_footprint"/>`
  `<param name="global_frame_id" value="map"/>`


    <!-- If you choose to define initial pose here -->
    <param name="initial_pose_x" value="5"/>
    <param name="initial_pose_y" value="5"/>/
`</node>`
 `<node name="move_base" pkg="move_base" type="move_base" respawn="false" output="screen">`
      `<param name="base_global_planner" value="navfn/NavfnROS" />`
      `<param name="base_local_planner" value="base_local_planner/TrajectoryPlannerROS"/>` 

  `<rosparam file="/home/workspace/catkin_ws/src/my_robot/config/costmap_common_params.yaml" command="load" ns="global_costmap" />`
  `<rosparam file="/home/workspace/catkin_ws/src/my_robot/config/costmap_common_params.yaml" command="load" ns="local_costmap" />`
  `<rosparam file="/home/workspace/catkin_ws/src/my_robot/config/local_costmap_params.yaml" command="load" />`
  `<rosparam file="/home/workspace/catkin_ws/src/my_robot/config/global_costmap_params.yaml" command="load" />`
  `<rosparam file="/home/workspace/catkin_ws/src/my_robot/config/base_local_planner_params.yaml" command="load" />`

`</node>`

`</launch>`





![image-20200829231544412](image-20200829231544412.png)	

This project uses a [Gazebo](http://gazebosim.org/#features) simulation platform to create a mobile Robot with C++ Nodes in ROS to chase white colored balls!!

[See the video demonstration](./robot1.mp4)

## **Robot Xacro file**

The Robot xacro file is responsible for the robot design and is located on GazeboROSFirstRobot\my_robot\urdf\my_robot.xacro

You could   modify the Robot design on https://mymodelrobot.appspot.com/

The Robot has one camera and one Lidar sensor

![image-20200829232538734](image-20200829232538734.png)





## Prerequisites/Dependencies

- Linux 16.04
- Gazebo >= 7.0
- ROS Kinetic
- make >= 4.1
- gcc/g++ >= 5.4

## Install ROS

Follow the steps in http://wiki.ros.org/kinetic/Installation

## Clone the repo in your workspace

```
$ git clone https://github.com/naubergois/GazeboROSFirstRobot.git
```

## Build the project

In the project folder
`$ cd catkin_ws`
`$ catkin_make`

## Launch the world

```
$ source devel/setup.bash`
$ roslaunch my_robot world.launch
```

## Launch the drive_bot and process_image nodes

Open a new terminal
`$ source devel/setup.bash`
`$ roslaunch ball_chaser ball_chaser.launch`

## Run the camera viewer

Open a new terminal
`$ source devel/setup.bash`
`$ rosrun rqt_image_view rqt_image_view`

## Test the robot

Move the white ball into the robot field of view and see the robot chasing the ball

## Project Description

Directory Structure

```
.Project2                          # Go Chase It Project
├── my_robot                       # my_robot package (differential drive)                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── robot_description.launch
│   │   ├── world.launch
│   ├── meshes                     # meshes folder for sensors
│   │   ├── hokuyo.dae
│   ├── urdf                       # urdf folder for xarco files
│   │   ├── my_robot.gazebo
│   │   ├── my_robot.xacro
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
├── my_cart                        # my_cart package (skid steer)                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── robot_description.launch
│   │   ├── world.launch
│   ├── meshes                     # meshes folder for sensors
│   │   ├── hokuyo.dae
│   ├── urdf                       # urdf folder for xarco files
│   │   ├── my_robot.gazebo
│   │   ├── my_robot.xacro
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
├── my_world                       # my_world package                   
│   ├── rviz                       # rviz folder for rviz config files
│   │   ├── config_file.rviz       # base rviz config
│   ├── models                     # models folder for world models
│   │   ├── my_<color>_ball        # <color> ball model
│   │   │   ├── model.config   
│   │   │   ├── model.sdf
│   ├── world                      # world folder for world files
│   │   ├── 1floorHouse.world
│   ├── launch                     # launch folder for launch files   
│   │   ├── my_cart_world.launch   # launch world with my_cart platform
│   │   ├── my_robot_world.launch  # launch world with my_robot platform
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
├── ball_chaser                    # ball_chaser package                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── ball_chaser.launch
│   ├── src                        # source folder for C++ scripts
│   │   ├── drive_bot.cpp
│   │   ├── process_images.cpp
│   ├── srv                        # service folder for ROS services
│   │   ├── DriveToTarget.srv
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info                  
└──           
```



