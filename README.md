# RoboCup 2022 Rescue Simulation Virtual Robot League (RSVRL)

This year, the  <strong>rescue virtual robot league</strong> infrastructure is migrated to <strong>ROS 2 foxy-version</strong>. All sections and packages are undergoing major changes, and we will provide practical demos of each major challenge in the league to better understand the new infrastructure according to the following program:


| Task                    | Goal                         | Progress                                               | 
|---                      | ---------------              | ---------------                                        |
|P3AT Single-Robot Spawn  |Immigration to ROS 2          |   ![100%](https://progress-bar.dev/100?title=completed)|
|P3AT Multi-Robot Spawn   |Immigration to ROS 2          |   ![85%](https://progress-bar.dev/85?title=progress)   |
|Wiki                     |Prepapre a e-manual for ROS 2 |   ![20%](https://progress-bar.dev/20?title=progress)   |
|SLAM & Mapping           |Prepapre a Demo for ROS 2     |   ![80%](https://progress-bar.dev/80?title=progress)   |
|Navigation               |Prepapre a Demo for ROS 2     |   ![1%](https://progress-bar.dev/1?title=progress)     |
|Exploration              |Prepapre a Demo for ROS 2     |   ![1%](https://progress-bar.dev/1?title=progress)     |
|Multi-Robot Map          |Prepapre a Demo for ROS 2     |   ![1%](https://progress-bar.dev/1?title=progress)     |
|2021 Maps                |Import world file to ROS 2    |   ![20%](https://progress-bar.dev/20?title=progress)   |
|Arial Robots             |Immigration to ROS 2          |   ![1%](https://progress-bar.dev/1?title=progress)     |


## Requirements
The requirement for preparing the environment of virtual rescue robots are as follows:
- [Ubuntu 20.04 LTS Desktop](https://releases.ubuntu.com/20.04/)
- [ROS 2 Foxy Version](https://docs.ros.org/en/foxy/index.html)

## Installation
1. <strong> Download and Install Ubuntu on your PC </strong>
    1. Dowload the [Ubuntu 20.04 LTS Desktop](https://releases.ubuntu.com/20.04/) image.
    2. Install the Ubuntu 20.04 on your PC by the following instruction from [ this link ](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview).

2. <strong> Install ROS 2 foxy version</strong> 
    
    Open a terminal console with Ctrl+Alt+T and enter belllow commands one at a time. 
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
    6. Set up your environment by sourcing the following file
    ```
    source /opt/ros/foxy/setup.bash
    ```

## Getting Started
This section will be updated.

## Wiki
This section will be updated.


