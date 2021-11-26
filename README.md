# RoboCup 2022 Rescue Simulation Virtual Robot League (RSVRL)

This year, the virtual rescue robot league infrastructure will migrate to ROS 2 foxy-version. All sections and packages are undergoing major changes, and we will provide practical examples of each major challenge in the league to better understand the new infrastructure according to the following program:


|Project | Progress | 
|--- | --------------- | 
|Single Robot Demo for ROS 2  | [###############################] 100% |
|Multi Robot Demo for ROS 2 | [###############################] 100% |
|Import Previous Maps to ROS 2 | [####] 20% |
|Virtual Robot Wiki  | [####] 20% |
|SLAM & Mapping Demo with ROS 2  | [#] 0% |
|Navigation Demo with ROS 2  | [#] 0% |
|Exploration Demo with ROS 2  | [#] 0% |
|Arial Robots for ROS 2  | [#] 0% |


## Requirements
The requirement for preparing the environment of virtual rescue robots are as follows:
- [Ubuntu 20.04 LTS Desktop](https://releases.ubuntu.com/20.04/)
- [ROS 2 Foxy Version](https://docs.ros.org/en/foxy/index.html)

## Installation
1. Download and Install Ubuntu on your PC
    1. Dowload the [Ubuntu 20.04 LTS Desktop](https://releases.ubuntu.com/20.04/) image.
    2. Install the Ubuntu 20.04 on your PC by the following instruction from [ this link ](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview).

2. Install ROS 2 foxy version:
    Open a terminal console with Ctrl+Alt+T and enter belllow commands one at a time. 
    ```
    sudo apt-get update
    sudo apt-get upgrade
    wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
    sudo chmod 755 ./install_ros2_foxy.sh
    bash ./install_ros2_foxy.sh
    ```
    If the above installation fails, please refer to 
    [the official ROS2 Foxy installation guide.](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

3. Install Dependent ROS 2 Packages:
    1. Open the terminal with Ctrl+Alt+T from Remote PC.
    2. Install Gazebo11
    ```
    sudo apt-get install ros-foxy-gazebo-*
    ```
    3. Install Cartographer
    ```
    sudo apt install ros-foxy-cartographer
    sudo apt install ros-foxy-cartographer-ros
    ```
    4. Install Navigation 2
    ```
    sudo apt install ros-foxy-navigation2
    sudo apt install ros-foxy-nav2-bringup
    ```

## Getting Started
This section will be updated.

## Wiki
This section will be updated.


