INTRODUCTION
================================

Development of a software architecture for the control of a mobile robot. General idea to control a mobile robot in the gazebo area by different methods. The following ways to do this are divided into three parts:

1. Use the coordinates entered by the user 
2. Driving robots with keyboards
3. Drive robots through user, avoid going against obstacles
Developed code written in ros area in python language, it is possible to develop similar codes with the same concept in C++ language

Materials and Methods
=========================

To make the following codes work on your system, you need to install the slam_gmapping package form: https://github.com/CarmineD8/slam_gmapping .You have to use the version adapted for your type of ros there are two types: 1.Kinetic and 2.Noetic in my case the second was used. The package can be downloaded from the following repository or by using fork directly in your ros workspace.

Actionlib stack was used in the developed code. Therefore you have to install actionlib in your repository, in case of lack of installation you can receive errors of type lack of attribution in the SimpleActionCliente or any type of attribution error on actionlib stack. For actionlib installation package to have more information can be found on: http://wiki.ros.org/actionlib.

# ros_gmapping_movebase
This ROS package provides three behaviors for controlling  a simulated mobile robot using gmapping and movebase rospackages.
You can get to know more about the detatils of the source code using the [documentation](https://github.com/samiur154/ASSIGNMENT-3-NEW-) provided for this rospackage.

## Installing and runnning 
Here is the instruction for using the package:
```bashscript
$ mkdir -p catkin_ws/src
```
```bashscript
$ cd catkin_ws/src
```
```bashscript
$ git clone https://github.com/samiur154/ASSIGNMENT-3-NEW-
```
```bashscript
$ cd ..
```
```bashscript
$ source /opt/ros/<distro>/setup.bash
```
```bashscript
$ catkin_make
```
```bashscript
$ source devel/setup.bash
```
```bashscript
$ sudo apt install konsole
```
```bashscript
$ roslaunch final_assignment simulation_gmapping.launch
```
in order to run the movebase node, open a new terminal in the same directory and run the following commands:
```bashscript
$ source devel/setup.bash
```
```bashscript
$ roslaunch final_assignment move_base.launch
```
For initializing the master node also, in a new terminal run the following commands:
```bashscript
$ source devel/setup.bash
```
```bashscript
$ roslaunch final_assignment master.launch
```
then you can choose robot's behaviour by inputing the corresponding number

Flowchart(General idea behind the program)
----------------------
Structure of the first part
![Immagine 2022-01-26 202719](https://user-images.githubusercontent.com/80394968/151232902-d5e477ba-b2c5-4b06-aea0-e8fb43b75c78.jpg)

Structure of the second and third part
![Immagine 2022-01-26 202822](https://user-images.githubusercontent.com/80394968/151233142-af56c67a-9ed2-47b2-88bd-e73f8d451c0e.jpg)

## Software Architecture
![Slide1](https://user-images.githubusercontent.com/65722399/147779264-a4f65968-3760-4857-8270-8b281d62693d.JPG)
Software architecture in this project is based on three nodes:
1. Master Node: 
        gets user request to choose robot behaviour using robot_state rosparam
2. Movebase Client Node: 
        gets desired position from user and sends it to movebase node using actionlib and if the goal is not reached before timeout cancels it
3. Teleop Twist Keyboard Node: it can implement two behaviours on robot:
        1: moving without obstacle avoidance:
            user can move the robot using keys it publishes the desired movements
            to cmd_vel topic
        2: moveing with obstacle avoidance:
            subscribes scan topic and uses it to detect obstacles. user 
            can move the robot using keys and it also avoids the robot
            from colliding the obstacles


## Pseudocode
```
***
  master node:
  
  main()
  {
        set the initial state of robot_state rosparam as 0 (master node)
        initialize the master node
        while (1)
        {
              if robot_state is 0 (master node)
              {
                    get user request for selecting the robot behaviour
                    set robot_state rosparam as the chosen robot behaviour to enable the corresponding node
              } 
              else 
              {
                    wait for other nodes' response 
              }
        }
  }

  
***
  movebase client node:
  
  main()
  {
        initialize movebase client node
        while(1)
        {
              get robot_state rosparam to detect user choice
              if robot_state is 1 (movebase_client node)
              {
                    get target point from user
                    call movebase node to send target point as an action 
                    detect if the target has been reached before timeout
              } 
              else 
              {
                    wait for master node's response
              }
        }
  }

***
  teleop twist keyboard node:
  
  main()
  {
        initialize teleop twist keyboard node
        while(1)
        {
              get robot_state rosparam to detect user choice
              if robot_state is 2 (manual teleop twist keyboard)
              {
                    get user command for moving the robot
                    publish the corresponding twist to cmd_vel topic
              } 
              else if  robot_state is 3 (assisted teleop twist keyboard)
              {
                    subscribe laser scan topic to detect obstacles
                    get user command for moving the robot
                    if there is an obstacle 
                    {
                          don't move the robot toward it
                    } 
                    else 
                    {
                          publish the corresponding twist to cmd_vel topic
                    }
              } 
              else 
              {
                    wait for master node's response
              }
        }
  }
 
```
