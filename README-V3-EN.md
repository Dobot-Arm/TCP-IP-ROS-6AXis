# <center>CR5Robot</center>

Chinese version of the README -> please [click here](./README-CN.md)

# Building
## ubuntu16.04

```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/CR5_ROS.git -b kinetic-devel

cd $HOME/catkin_ws

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```

## ubuntu18.04

### Use git to clone the source code
```
cd $HOME/catkin_ws/src
git clone https://github.com/Dobot-Arm/CR5_ROS.git -b melodic-devel
cd $HOME/catkin_ws
```

### building
```
catkin_make
```

### activate this workspace
```
source $HOME/catkin_ws/devel/setup.bash
```
# set the dobot type
### If you use CR3 robot， please type the fllow commands
```
echo "export DOBOT_TYPE=cr3" >> ~/.bashrc
source ~/.bashrc
```
### If you use CR5 robot， please type the fllow commands
```
echo "export DOBOT_TYPE=cr5" >> ~/.bashrc
source ~/.bashrc
```
### If you use CR10 robot， please type the fllow commands
```
echo "export DOBOT_TYPE=cr10" >> ~/.bashrc
source ~/.bashrc
```
### If you use CR16 robot， please type the fllow commands
```
echo "export DOBOT_TYPE=cr16" >> ~/.bashrc
source ~/.bashrc
```

# Example Demonstration

## Apply in simulation environment

1. ## rviz display

    ```
    roslaunch dobot_description display.launch
    ```

    User can adjust the angle of each joint by joint_state_publisher_gui, and see the result from rviz

    ![rviz display](./rviz.jpg)


2. ## moveit control
    * Active moveit by the following commands
    ```
    roslaunch dobot_moveit demo.launch
    ```
    * Drag the joint to any direction, then click "Plan and Excute" to see the result

    ![moveit display](./moveit.gif)

3. ## Gazebo simulated
    * Start Gazebo with the following command
    ```
    roslaunch dobot_gazebo gazebo.launch 
    ```
    * Again, you can use MoveIt!  Control the gazebo robots
    * For setting up the MoveIt! nodes to allow motion planning run.The dobot type needs to correspond
    ```
    roslaunch cr5_moveit cr5_moveit_planning_execution.launch  sim:=True
    ```
    * For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run.The dobot type needs to correspond
    ```
    roslaunch cr5_moveit moveit_rviz.launch config:=True
    ```
    * Drag the joint to any direction, then click "Plan and Excute" to see the result
## Controlling real robotic arm

* **Connect the robotic arm with following command, and robot_ip is the IP address that the real arm locates**
    ```
    roslaunch dobot_bringup bringup.launch robot_ip:=192.168.5.1
    ```

* **Call Rosservice, eg: call EnableRobot**
    ```
    rosservice call /dobot_v4_bringup/srv/EnableRobot "{}"
    ```

* **Active Moveit with following command**
    ```
    roslaunch dobot_moveit moveit.launch
    ```

* **Install DobotControl Plugin to enable the robotic arm**
    
    1. Press Panels on the tool bar of rviz --> "Add New Panel"
    2. Choose DobotControl, then press "OK"
    3. Press "EnableRobot" to enable the arm
    4. When "Connected" and "Enable" is displayed on the status bar, it means the robotic arm is connected and enabled, and users can control the robotic arm via Moveit

    ![DobotControl](./cr5control.jpg)


# Custom Function Development

    Msg and srv is defined in dobot_bringup. Users can control the robotic arm via those underlying commands
