# miniBot_RoboRTS

-   ROS Melodic
-   ROS workspace

## Packages

-   ros-melodic-joy
-   ros-melodic-teleop-twist-joy
-   ros-melodic-telop-twist-key
-   roborts_base
-   roborts_msgs
    [RoboMaters/RoboRTS](https://github.com/RoboMaster/RoboRTS.git)

## Running Teleop

### Teleop Joy

-   Tested on Logitech F3710

```
rosrun joy joy_node
rosrun teleop_twist_joy teleop_node
```
source: [ros-wiki-teleop-twist-joy](http://wiki.ros.org/teleop_twist_joy)
source: [ros-wiki-joy](http://wiki.ros.org/joy)

### Keyboard

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
source: [ros-wiki-teleop-twist-key](http://wiki.ros.org/teleop_twist_keyboard)

## Setup

-   turn off desktop environment
    `sudo systemctl stop gdm`
