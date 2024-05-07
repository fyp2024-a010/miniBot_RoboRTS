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
    `sudo systemctl stop gdm` (turns off the gnome desktop environment)
-   alternatively, `sudo systemctl set-default multi-user.target` (sets terminal as default boot target)
-   set user dialout so that permission is granted for serial ports
    `sudo usermod -a -G dialout your_username`
-   making script executable
    `chmod +x mini_bot_base_node.py`

### Peripheral port setup - roborts

[robomaster.gitub.io](https://robomaster.github.io/RoboRTS-Tutorial/#/en/quick_start/setup_on_manifold2?id=peripheral-port-mapping)

```
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", SYMLINK+="serial_sdk"
```

## Tools

-   copying files over ssh: `scp [OPTION] [user@]SRC_HOST:]file1 [user@]DEST_HOST:]file2
` e.g. `scp -r miniBot minibot@192.168.0.104:/home/minibot`
