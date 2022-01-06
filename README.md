# RoboCup 2022 Rescue Simulation Virtual Robot League (RSVRL)

This year, the  <strong>rescue virtual robot league</strong> infrastructure is migrated to <strong>ROS 2 foxy-version</strong>. All sections and packages are undergoing major changes, and we will provide practical demos of each major challenge in the league to better understand the new infrastructure according to the following program:

    
| Task                                                        | Progress                                                 | 
| :-----------------:                                         | :----------------:                                   |
|P3AT Robot: Import Robot Models/Sensors to ROS2              | ![100%](https://progress-bar.dev/100?title=done)    |
|P3AT Robot: Multi-Robot Scenario  for RO2                    | ![100%](https://progress-bar.dev/100?title=done)    |
|P3AT Robot: Single-Robot SLAM & Mapping Demo for ROS2        | ![100%](https://progress-bar.dev/100?title=done)     |
|World Maps: Import Former Maps to ROS 2                      | ![100%](https://progress-bar.dev/100?title=done)    |
|Wiki: Wiki for Virtual Robot Community-Release 1             | ![90%](https://progress-bar.dev/90?title=progress)       |
|P3AT Robot: Single-Robot Navigation Demo for ROS2            | ![70%](https://progress-bar.dev/70?title=progress)       |
|P3AT Robot: Single-Robot Exploration Demo for ROS2           | ![10%](https://progress-bar.dev/10?title=progress)       |
|P3AT Robot: Multi-Robot Map-Merge Demo for ROS2              | ![0%](https://progress-bar.dev/0?title=to-do)         |
|Wiki: Wiki for Virtual Robot Community-Release 2             | ![0%](https://progress-bar.dev/0?title=to-do)       |
|Arial Robot: Import Robot Models/Sensors to ROS2             | ![0%](https://progress-bar.dev/0?title=to-do)         |
|Crawler Robot: Import Robot Models/Sensors to ROS2           | ![0%](https://progress-bar.dev/0?title=to-do)         |
|Wiki: Wiki for Virtual Robot Community-Release 3             | ![0%](https://progress-bar.dev/0?title=to-do)       |
    

## Requirements
The requirement for preparing the environment of virtual rescue robots are as follows:
- [Ubuntu 20.04 LTS Desktop](https://releases.ubuntu.com/20.04/)
- [ROS 2 Foxy Version](https://docs.ros.org/en/foxy/index.html)

## Installation
1. <strong> Download and Install Ubuntu on your PC </strong>
    1. Dowload the [Ubuntu 20.04 LTS Desktop](https://releases.ubuntu.com/20.04/) image.
    2. Install the Ubuntu 20.04 on your PC by the following instruction from [ this link ](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview).

2. <strong> Install ROS 2 foxy version</strong> 
    
    Open a terminal console with Ctrl+Alt+T and enter the following commands one at a time. 
    ```
    sudo apt update && sudo apt install curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo sed -i -e 's/ubuntu .* main/ubuntu focal main/g' /etc/apt/sources.list.d/ros2.list
    ```
    If the above installation fails, please refer to 
    [the official ROS2 Foxy installation guide.](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

3. <strong> Install Dependent ROS 2 Packages </strong>
    1. Open the terminal with Ctrl+Alt+T from Remote PC.
    2. Install Base Packages
    ```
    sudo apt update
    sudo apt install ros-foxy-desktop
    sudo apt install ros-foxy-ros-base
    ```
    3. Install Gazebo11
    ```
    sudo apt-get install ros-foxy-gazebo-*
    ```
    4. Install Cartographer
    ```
    sudo apt install ros-foxy-cartographer
    sudo apt install ros-foxy-cartographer-ros
    ```
    5. Install Navigation 2
    ```
    sudo apt install ros-foxy-navigation2
    sudo apt install ros-foxy-nav2-bringup
    ```
    6. Add sourcing to your shell startup script by sourcing the setup.bash file 
    ```
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    ```

4. <strong> Set up RoboCup2022RVRL_Demo project </strong>
    1. Clone RoboCup2022RVRL_Demo project into the your home directory
    ```
    cd ~
    git clone https://github.com/RoboCup-RSVRL/RoboCup2022RVRL_Demo.git
    ```
    2. Build RoboCup2022RVRL_Demo project
    ```
    cd RoboCup2022RVRL_Demo
    colcon build
    ```
    3. Add sourcing to RoboCup2022RVRL_Demo shell startup script by sourcing the setup.bash file 
    ```
    echo "source /home/{your username}/RoboCup2022RVRL_Demo/install/setup.bash" >> ~/.bashrc
    ```
    
## Getting Started
<strong> For getting strated, we launch a sample map with three robots(p3at) as following items: </strong>
1. Open a terminal console with Ctrl+Alt+T and launch the house_map world map
```
ros2 launch rvrl_gazebo house_map.launch.py
```
As shown in this example, three robots are spawned in the environment, each of them can be controlled manually with the following procedure.
    
2. Open terminal 1 with Ctrl+Alt+T to drive the robot1 with following command:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=robot1/cmd_vel
```
3. Open terminal 2 with Ctrl+Alt+T to drive the robot2 with following command: 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=robot2/cmd_vel
```
4. Open terminal 3 with Ctrl+Alt+T to drive the robot3 with following command:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=robot3/cmd_vel
```
5. Open terminal 4 with Ctrl+Alt+T to get the robots image camera with following command:
```
ros2 run rqt_image_view rqt_image_view
```
## SLAM Demo
1. In this demo, we use the cartographer package for SLAM demo. After driving the robots, then create the SLAM node for each robots using the following command:
```
ros2 launch rvrl_cartographer cartographer.launch.py robot_name:=robot1
```

Here you can change the robot_name:='robotname' to create different SLAM nodes for other robots.

2. Open rviz2 to visualize robot's map using the following command:
```
ros2 run rviz2 rviz2
```


## Useful Links

* [Rescue Virtual Robot Wiki](https://robocup-rsvrl.github.io/)
* [RoboCup Federation official Website](https://www.robocup.org/)
* [Virtual Robot League Official Website](https://rescuesim.robocup.org/competitions/virtual-robot-competition/)
* [Virtual Robot League in Robocup](https://www.robocup.org/leagues/27)
* [Official Website Robocup 2022](https://2022.robocup.org/)







