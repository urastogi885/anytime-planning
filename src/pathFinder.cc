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

PathFinder::PathFinder(uint16_t xs, uint16_t ys, uint16_t xg, uint16_t yg, string robot_world_loc) {
    start_x = xs;
    start_y = ys;
    goal_x = xg;
    goal_y = yg;
    robot_world = cv::imread(robot_world_loc);
}

int8_t PathFinder::FindPathToGoal() {
    priority_queue<vector<float>, vector<vector<float>>, MinHeapComparator> queue_nodes;

    vector<float> node1 = {50, 30, 40.6};
    vector<float> node2 = {20, 30, 40.6};
    vector<float> node3 = {50, 30, 41.6};

    queue_nodes.push(node1);
    queue_nodes.push(node2);
    queue_nodes.push(node3);

    PrintVector(queue_nodes.top());
    return 0;
}

void PathFinder::PrintVector(vector<float> vec) { 
    for (int i = 0; i < vec.size(); i++) { 
        cout << vec[i] << " "; 
    } 
    cout << endl;
    return;
} 

PathFinder::~PathFinder() {
}
