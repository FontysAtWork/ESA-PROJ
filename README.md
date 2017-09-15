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
After you've cloned this repository, you might notice things will not build straight away. You'll encounter the following errors:  

1. protobuf_comm
2. brics_actuator
3. convex_decomposition & ivcon
4. protobuf compile errors

To fix them:

1. Install [atwork_refbox_comm](https://github.com/industrial-robotics/atwork_refbox_comm) according to the instructions on the linked repo.
2. Clone [brics_actuator](https://github.com/wnowak/brics_actuator) into `src/youbot_fontys` if it doesn't exist there already.
3. Convex_decomposition needs more things:
  1. Clone [pr2_common](https://github.com/pr2/pr2_common) into `src/` if it doesn't exist there already.
  2. Clone into `src/pr2_common`: [convex_decomposition](https://github.com/ros/convex_decomposition)
  3. Clone into `src/pr2_common`: [ivcon](https://github.com/ros/ivcon)
  4. Install the following packages `sudo apt install ros-indigo-ivcon && ros-indigo-convex-decomposition`
4. To install the protobuf library we need te execute the following tasks:
  1. Install the following packages `sudo apt-get install autoconf automake libtool curl make g++ unzip`
  2. Clone [protobuf](https://github.com/google/protobuf) in a folder of your own choice.
  3. Execute the following commands:
  ```
  ./autogen.sh  
  ./configure  
  make  
  make check  
  sudo make install  
  sudo ldconfig # refresh shared library cache.  
  ```

## Building

Travis CI should take care of it.

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
