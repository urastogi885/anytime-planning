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
    parent_nodes = cv::Mat::ones(robot_world_size[1], robot_world_size[0], CV_64FC1);
    for (int y = 0; y < robot_world_size[1]; ++y) {
        for (int x = 0; x < robot_world_size[0]; ++x) {
            parent_nodes.at<double>(y, x) = kNoParent;
        }
    }
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

    parent_nodes.at<double>(robot_start_pos[1], robot_start_pos[0]) = kStartParent;

    while (!queue_nodes.empty()) {
        Node current_node = queue_nodes.top();
        queue_nodes.pop();
        if (current_node.x == robot_goal_pos[0] && current_node.y == robot_goal_pos[1]) {
            error_logger.Log("Path to goal found!", kDebug);
            return true;
        }
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_node.x, i);
            uint16_t y = actions.GetNextCoord(current_node.y, i, 'y');

            if (IsNodeValid(x, y) && (int64)parent_nodes.at<double>(y, x) == kNoParent) {
                costs[0] = CostToCome(current_node.cost_to_come, i);
                costs[1] = CostToGo(x, y, 1) + costs[0];
                queue_nodes.push(Node(x, y, costs));
                parent_nodes.at<double>(y, x) = RavelIndex(current_node.x, current_node.y);
            }
        }
    }

    error_logger.Log("Path to goal NOT found!", kDebug);

    return false;
}

bool PathFinder::GeneratePathList() {
    std::vector<std::pair<uint16_t, uint16_t>> path_nodes;
    std::pair<uint16_t, uint16_t> last_node = std::make_pair(robot_goal_pos[0], robot_goal_pos[1]);
    path_nodes.push_back(last_node);
    std::cout << last_node.first << ", " << last_node.second << std::endl;

    while ((int64)parent_nodes.at<double>(last_node.second, last_node.first) != kStartParent) {
        last_node = UnravelIndex((int64)parent_nodes.at<double>(last_node.second, last_node.first));
        std::cout << last_node.first << ", " << last_node.second << std::endl;
        path_nodes.push_back(last_node);
    }
    return true;
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

uint32_t PathFinder::RavelIndex(uint16_t pos_x, uint16_t pos_y) {
    return (pos_y * robot_world_size[0]) + pos_x;
}

std::pair<uint16_t, uint16_t> PathFinder::UnravelIndex(uint32_t identifier) {
    return std::make_pair((int)identifier%robot_world_size[0], (int)identifier/robot_world_size[0]);
}

PathFinder::~PathFinder() {
}
