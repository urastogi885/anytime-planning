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
- The above commands are based on Ubuntu 18.04. They might differ for other OS-based machines such as Windows and MacOC.

## Build & Run
- Clone the repository into your workspace and move into the project directory
```
git clone https://github.com/urastogi885/anytime-planning
cd anytime-planning/
```

- Configure the project and run it
```
bash explorer.sh
python main.py <start_x,start_y> <goal_x,goal_y> <robot_radius, clearance> <method> <inflation_factor>
```

- Currently, the methods can be referenced in the following way:
```
0 - A*
1 - Anytime A* (ATA*)
2 - Anytime Repairing A* (ARA*)
3 - Anytime Nonparametric A* (ANA*)
```

- For instance, to execute ATA*, you can use the following command:
```
python main.py 50,30 150,150 3,2 1 3.5
```
