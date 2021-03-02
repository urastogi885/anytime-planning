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

PathFinder::PathFinder(uint16_t start_x, uint16_t start_y, uint16_t goal_x, uint16_t goal_y, std::string robot_world_loc) {
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
    if (!(IsNodeValid(robot_start_pos[0], robot_start_pos[1]) && IsNodeValid(robot_goal_pos[0], robot_goal_pos[1]))) {
        error_logger.Log("Start or goal position is in obstacle space!", kDebug);
        return false;
    }

    error_logger.Log("Finding path to goal node...", kDebug);
    std::priority_queue<Node, std::vector<Node>, CompareCostToCome> queue_nodes;

    float costs[2] = {0, -1};
    queue_nodes.push(Node(robot_start_pos[0], robot_start_pos[1], costs));

    cv::Mat parent_nodes = cv::Mat::zeros(robot_world_size[1], robot_world_size[0], CV_8U);
    parent_nodes.at<uchar>(robot_start_pos[1], robot_start_pos[0]) = kStartParent;
    // uint32_t counter = 0;
    // uint32_t prev = 0;

    while (!queue_nodes.empty()) {
        Node current_node = queue_nodes.top();
        queue_nodes.pop();
        // ++counter;
        // if (counter - prev == 500) {
        //     prev = counter;
        //     error_logger.Log("(" + std::to_string(current_node.x) + ", " + std::to_string(current_node.y) + ", " + std::to_string(current_node.total_cost) + ")", kInfo);
        // }
        if (current_node.x == robot_goal_pos[0] && current_node.y == robot_goal_pos[1]) {
            error_logger.Log("Path to goal found!", kDebug);
            return true;
        }
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_node.x, i);
            uint16_t y = actions.GetNextCoord(current_node.y, i, 'y');

            if (IsNodeValid(x, y) && (int)parent_nodes.at<uchar>(y, x) == kNoParent) {
                costs[0] = CostToCome(current_node.cost_to_come, i);
                costs[1] = CostToGo(x, y, 1) + costs[0];
                queue_nodes.push(Node(x, y, costs));
                parent_nodes.at<uchar>(y, x) = kParent;
            }
        }
    }

    error_logger.Log("Path to goal not found!", kDebug);

    return false;
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

float PathFinder::CostToCome(float parent_node_cost, uint8_t action) {
    if (action < kUpRight) {
        return parent_node_cost + 1;
    }

    return parent_node_cost + sqrt(2);
}

float PathFinder::CostToGo(uint16_t pos_x, uint16_t pos_y, float epsilon) {
    return epsilon * sqrt(pow(robot_goal_pos[0] - pos_x, 2) + pow(robot_goal_pos[1] - pos_y, 2));
}

PathFinder::~PathFinder() {
}
