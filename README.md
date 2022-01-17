# Anytime Planning
[![Build Status](https://travis-ci.org/urastogi885/anytime-planning.svg?branch=main)](https://travis-ci.org/urastogi885/anytime-planning)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/urastogi885/anytime-planning/blob/main/LICENSE)

## Overview
The project implements various anytime motion planning algorithms such as ATA* and ANA*.

<p align="center">
  <img src="https://github.com/urastogi885/anytime-planning/blob/main/images/readme/ata_star_combine.jpg">
  <br><b>Figure 1 - Improvement in path provided by Anytime A* (ATA*)</b><br>
</p>

In the figure above, we can see that the path provided by ATA* improves over the iterations (left-to-right). From left-to-right, the left one is the first the alogrithm outputs while right one is last path the algorithm outputs.

## Dependencies
- Languages: `C++11, Python`
- CMake: `v3.2.0 (at least)`
- OpenCV: `v3.2.0 (at least)`
- Python libraries: `NumPy, Shutil, Glob, Opencv-Python`

## Install Dependencies
- Install necessary depenedencies using:
```
sudo apt-get install build-essential cmake libopencv-dev
python -m pip install numpy shutil glob opencv-python
```
- OpenCV can be installed from scratch as well but is not necessary for this project. One of the resources is [here](https://learnopencv.com/install-opencv-4-on-ubuntu-18-04/).
- The above commands are based on Ubuntu 18.04. They might differ for other OS-based machines such as Windows and MacOS.

## Build & Run
- Clone the repository into your workspace and move into the project directory
```
git clone https://github.com/urastogi885/anytime-planning
cd anytime-planning/
```
- Run the project
```
bash run_anytime_planning.sh
```
- Arguments in the shell script
```
-h  show this help text

-x  set the start x-coordinate (default: 50, type: int)

-y  set the start y-coordinate (default: 30, type: int)

-a  set the goal x-coordinate (default: 150, type: int)

-b  set the goal y-coordinate (default: 150, type: int)

-r  set radius of the robot (default: 3, type: int)

-c  set clearance required between the robot and obstacles (default: 2, type: int)

-m  set method the robot uses to find path from start to goal (default: 0, , type: int)
    Use 0: A*, 1: ATA*, 2: ARA*, 3: ANA*
    
-i  set inflation factor for the method (default: 1.0, type: float, minimum: 1.0)
```
- Note: The world size is 300x200 so set the start and goal point accordingly.
