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

Perform a git clone recursively:  
`git clone --recursive https://github.com/minhtrietdiep/ESA-PROJ`  
`cd ESA-PROJ`
`git submodule update --init --recursive`

Install some packages:
```sh
sudo apt install ros-indigo-navigation ros-indigo-ivcon ros-indigo-convex-decomposition autoconf automake libtool curl make g++ unzip gksu
```

Errors:

1. convex_decomposition & ivcon
2. protobuf compile errors
3. gksudo missing

To fix them:

1. Convex decomposition needs more things:
   1. Install the following packages `sudo apt install ros-indigo-ivcon ros-indigo-convex-decomposition`

2. Set up `atwork_refbox_comm` since recursive git submodules might not have initialized properly
   1. `cd src/atwork_refbox_comm/`
   2. `git submodule init`
   3. `git submodule update`

3. To install the protobuf library we need te execute the following tasks:
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
4. Install the following package `sudo apt install gksu`

## Building

`cd ~/git/ESA-PROJ`  
`catkin_make`

## Running the simulator

For every new terminal window, you need to do this:

`source ./devel/setup.bash`  
`roslaunch youbot_gazebo_robot youbot.launch`

Alternatively you can just add `source ./devel/setup.bash` to your .bashrc.

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
