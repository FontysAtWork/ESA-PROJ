# ESA-PROJ

[![Build Status](https://travis-ci.org/minhtrietdiep/ESA-PROJ.svg?branch=master)](https://travis-ci.org/minhtrietdiep/ESA-PROJ)

This project will realize the architecture for the RoboCup@Work entry for the Fontys@Work team.

## Repository instructions

* Each new node gets a new folder in the repo root
* Each new node is a new branch
* A new node includes at least the following files for ROS to build automagically:
  * `./include/NODENAME_interface/*.h`
  * `./src/*.cpp`
  * `./test/*.cpp`
  * `./CMakeList.txt`
  * `./package.xml`
  * `./NODENAME_plugin.xml`

## Prerequisites

You'll need to set up the repo and pull in dependencies.

Install some packages:  
```sh
sudo apt install ros-indigo-navigation ros-indigo-ivcon ros-indigo-convex-decomposition ros-indigo-object-recognition-msgs ros-indigo-moveit-commander autoconf automake libtool curl make g++ unzip gksu
```

Perform a git clone recursively:  
`git clone --recursive https://github.com/minhtrietdiep/ESA-PROJ`  
`cd ESA-PROJ`
`git submodule update --init --recursive`

Set up `atwork_refbox_comm` since recursive git submodules might not have initialized properly
   1. `cd src/atwork_refbox_comm/`
   2. `git submodule init`
   3. `git submodule update`

Install the protobuf library. To install the protobuf library we need te execute the following tasks:
   1. Install the following packages `sudo apt install autoconf automake libtool curl make g++ unzip`
   2. Clone [protobuf](https://github.com/google/protobuf) in a folder of your own choice.
   3. `cd` into `protobuf`
   4. Execute the following commands:  
   ```
   ./autogen.sh  
   ./configure  
   make  
   make check  
   sudo make install  
   sudo ldconfig # refresh shared library cache.  
   ```

This can take a while, so sit back and make yourself another cup of coffee.

## Building

`cd ~/git/ESA-PROJ`  
`catkin_make`

Depending on how the gods are feeling, the build may succeed or fail. The most common problems and known fixes that shouldn't work but do work:

* Missing header: Remove `build` and `devel` folders, and try again.
* Some linker error or other vague error near the end: Restart the `catkin_make`.

## Running things

For every new terminal window, you need to do this:

`source ./devel/setup.bash`  

Alternatively you can just add `source ~/git/ESA-PROJ/devel/setup.bash` to your .bashrc, but it's not recommended if you have multiple ROS projects going on.

Simulator:  
`roslaunch youbot_gazebo_robot youbot.launch`

Mapping:
`tba`

Teleop:
`tba`

Navigation:
`tba`

Vision:
`tba`

Arm:
`tba`

## Project members

* Lars Jaeqx
* Hubert Heijnen
* Luuk van Rossum
* Wesley Sneijers
* Jasper de Weger
* Minh-Triet Diep

## Other related projects
* [Fontys' previous entry](https://github.com/Youbotfontysatwork/youbot_fontys)
* [Fontys' previous entry: Waypoints](https://github.com/BasB1/youbot_interface)
* [Competitor's entry](https://github.com/mas-group/robocup-at-work)

## Protip for VS Code

Just open this repository itself (ESA-PROJ) in VS Code as folder, the ROS plugin for VS Code should figure out things after trying a (succesful) catkin_make in the folder.
