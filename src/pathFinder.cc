/**
 * MIT License
 *
 * Copyright (c) 2021 Umang Rastogi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file pathFinder.cc
 * @author Umang Rastogi
 * @brief Implements the PathFinder's class to find a path from start to goal if it exists
 */

#include "pathFinder/pathFinder.h"

PathFinder::PathFinder(uint16_t start_x, uint16_t start_y, uint16_t goal_x, uint16_t goal_y, string robot_world_loc) {
    // Store start and goal position of the robot
    robot_start_pos[0] = start_x;
    robot_start_pos[1] = start_y;
    robot_goal_pos[0] = goal_x;
    robot_goal_pos[1] = goal_y;
    // Read the image and store its size
    robot_world = cv::imread(robot_world_loc, CV_LOAD_IMAGE_GRAYSCALE);
    robot_world_size[1] = robot_world.rows;
    robot_world_size[0] = robot_world.cols;
}

bool PathFinder::FindPathToGoal() {
    priority_queue<vector<float>, vector<vector<float>>, MinHeapComparator> queue_nodes;

    vector<float> node1 = {50, 30, 40.6};
    vector<float> node2 = {20, 30, 40.6};
    vector<float> node3 = {50, 30, 41.6};

    queue_nodes.push(node1);
    queue_nodes.push(node2);
    queue_nodes.push(node3);

    PrintVector(queue_nodes.top());

    return true;
}

void PathFinder::PrintVector(vector<float> vec) { 
    for (int i = 0; i < vec.size(); i++) { 
        cout << vec[i] << " "; 
    } 
    cout << endl;
    return;
}

bool PathFinder::IsNodeValid(uint16_t pos_x, uint16_t pos_y) {
    pos_y = robot_world_size[1] - pos_y;
    // Boundary and obstacle space check
    if (pos_x <= 0 || pos_x > robot_world_size[0] || pos_y <= 0 || pos_y > robot_world_size[1]) {
        return false;
    } else if ((int)robot_world.at<uchar>(pos_y, pos_x) == 0) {
        return false;
    }

    return true;
}

PathFinder::~PathFinder() {
}
