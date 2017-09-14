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
