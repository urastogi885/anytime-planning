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

#include <iostream>
#include <fstream>
#include "pathFinder/pathFinder.h"

PathFinder::PathFinder(uint16_t start_x, uint16_t start_y, uint16_t goal_x,
                uint16_t goal_y, const char * robot_world_loc) {
    // Store start and goal position of the robot
    robot_start_pos[0] = start_x;
    robot_start_pos[1] = start_y;
    robot_goal_pos[0] = goal_x;
    robot_goal_pos[1] = goal_y;
    // Read the image and store its size
    robot_world = cv::imread(robot_world_loc, CV_LOAD_IMAGE_GRAYSCALE);
    robot_world_size[1] = robot_world.rows;
    robot_world_size[0] = robot_world.cols;
    // Initialize parent nodes and cost maps
    uint32_t start_node_index = RavelIndex(robot_start_pos[0], robot_start_pos[1]);
    parent_nodes[start_node_index] = kStartParent;
    cost_to_come[start_node_index] = 0;
    final_cost[start_node_index] = CostToGo(robot_start_pos[0], robot_start_pos[1]);
    open_nodes_check_map[start_node_index] = true;
}

bool PathFinder::FindPathToGoal(uint8_t method, float epsilon) {
    switch (method) {
    case kAstar:
        logger.Log("Finding path to goal using A*...", kInfo);
        return Astar();

    case kAtaStar:
        logger.Log("Finding path to goal using ATA*...", kInfo);
        return AtaStar(epsilon);

    case kAraStar:
        logger.Log("Finding path to goal using ARA*...", kInfo);
        return AraStar(epsilon);

    case kAnaStar:
        logger.Log("Finding path to goal using ANA*...", kInfo);
        return AnaStar();

    default:
        logger.Log("INVALID method!", kFatal);
        return false;
    }
}

void PathFinder::GeneratePathList(std::map<uint32_t, int64_t> path_nodes, uint32_t list_index) {
    std::string file_name = "path/pathList_" + std::to_string(list_index) + "_.txt";
    std::ofstream path_list;
    path_list.open(file_name, std::ios::out | std::ios::trunc);

    std::pair<uint16_t, uint16_t> last_node = std::make_pair(robot_goal_pos[0], robot_goal_pos[1]);
    path_list << last_node.first << ", " << last_node.second << std::endl;

    int64_t node_index = kNoParent;

    // Backtrack the start node
    while (path_nodes[RavelIndex(last_node.first, last_node.second)] != kStartParent ||
            parent_nodes[RavelIndex(last_node.first, last_node.second)] != kStartParent) {
        if ((path_nodes % RavelIndex(last_node.first, last_node.second)) == kNoParent) {
            node_index = parent_nodes[RavelIndex(last_node.first, last_node.second)];
            logger.Log("Parent node not in path nodes!", kDebug);
        } else {
            node_index = path_nodes[RavelIndex(last_node.first, last_node.second)];
        }
        last_node = UnravelIndex(node_index);
        // last_node = UnravelIndex(path_nodes[RavelIndex(last_node.first, last_node.second)]);
        path_list << last_node.first << ", " << last_node.second << std::endl;
        if (last_node.first == 0 && last_node.second == 0)  break;
    }

    path_list.close();
    logger.Log("Path list GENERATED!", kDebug);

    return;
}

bool PathFinder::Astar() {
    // Initialize open nodes
    open_nodes.push_back(Node(robot_start_pos[0], robot_start_pos[1],
                        final_cost[RavelIndex(robot_start_pos[0], robot_start_pos[1])]));
    // Try finding path to goal until the queue goes empty
    while (!open_nodes.empty()) {
        // Extract the node with minimum cost
        Node current_node = open_nodes.front();
        open_nodes.erase(open_nodes.begin());
        open_nodes_check_map[RavelIndex(current_node.x, current_node.y)] = false;

        // Exit if goal is found
        if (current_node.x == robot_goal_pos[0] && current_node.y == robot_goal_pos[1]) {
            logger.Log("Path to goal FOUND!", kDebug);
            GeneratePathList(parent_nodes, 1);
            return true;
        }

        // Generate child nodes
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_node.x, i);
            uint16_t y = actions.GetNextCoord(current_node.y, i, 'y');

            uint32_t node_index = RavelIndex(x, y);
            double temp_cost_to_come = CostToCome((cost_to_come | RavelIndex(current_node.x, current_node.y)), i);

            // Make sure node is not in obstacle space and it is better then the previous one
            if (IsNodeValid(x, y) && temp_cost_to_come <  (cost_to_come | node_index)) {
                parent_nodes[node_index] = RavelIndex(current_node.x, current_node.y);
                cost_to_come[node_index] = temp_cost_to_come;
                final_cost[node_index] = temp_cost_to_come + CostToGo(x, y);
                if (!open_nodes_check_map[node_index]) {
                    open_nodes.push_back(Node(x, y, final_cost[node_index]));
                    open_nodes_check_map[node_index] = true;
                }
            }
        }
        // Get node with minimum cost n top
        std::make_heap(open_nodes.begin(), open_nodes.end(), CompareTotalCost());
    }

    logger.Log("Path to goal NOT FOUND!", kDebug);
    return false;
}

bool PathFinder::AtaStar(float epsilon) {
    // Initialize bound, inflation factor, and output counter
    double bound = INFINITY;
    uint32_t output_count = 0;
    bool path_found = false;

    // Initialize open nodes
    final_cost[RavelIndex(robot_start_pos[0], robot_start_pos[1])] *= epsilon;
    open_nodes.push_back(Node(robot_start_pos[0], robot_start_pos[1],
                            final_cost[RavelIndex(robot_start_pos[0], robot_start_pos[1])]));

    // Initialize a map to store closed nodes
    closed_nodes[RavelIndex(robot_start_pos[0], robot_start_pos[1])] = kStartParent;

    // Try finding path to goal until the queue goes empty
    while (!open_nodes.empty()) {
        // Extract the node with minimum cost
        Node current_node = open_nodes.front();
        open_nodes.erase(open_nodes.begin());
        uint32_t current_node_index = RavelIndex(current_node.x, current_node.y);
        open_nodes_check_map[current_node_index] = false;

        // Add extracted node to the list of closed nodes
        if (!(current_node.x == robot_start_pos[0] && current_node.y == robot_start_pos[1])) {
            closed_nodes[current_node_index] = parent_nodes[current_node_index];
        }

        // Exit if goal is found
        if (current_node.x == robot_goal_pos[0] && current_node.y == robot_goal_pos[1]) {
            path_found = true;
            logger.Log("Path to goal FOUND!", kDebug);

            // Update the bound and generate path text file
            bound = cost_to_come[current_node_index] + CostToGo(current_node.x, current_node.y);
            GeneratePathList(closed_nodes, ++output_count);

            // Prune the open nodes list
            logger.Log("Pruning open nodes list...", kInfo);
            for (auto it = std::begin(open_nodes); it != std::end(open_nodes); ++it) {
                Node node = *it;
                if (cost_to_come[RavelIndex(node.x, node.y)] + CostToGo(node.x, node.y) >= bound) {
                    open_nodes.erase(it);
                    open_nodes_check_map[RavelIndex(node.x, node.y)] = false;

                    // No more pruning possible
                    if (open_nodes.empty()) {
                        logger.Log("Open nodes list is EMPTY!", kWarn);
                        break;
                    }
                }
            }
            // If vector becomes empty during pruning exit
            if (open_nodes.empty()) {
                break;
            }
        }

        // Get neighbors to the current node
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_node.x, i);
            uint16_t y = actions.GetNextCoord(current_node.y, i, 'y');

            double temp_cost_to_come = CostToCome((cost_to_come | current_node_index), i);

            // Make sure node is not in obstacle space and it obeys the bound
            if (IsNodeValid(x, y) && temp_cost_to_come + CostToGo(x, y) < bound) {
                uint32_t node_index = RavelIndex(x, y);
                // Make sure node is not in obstacle space and it is better then the previous one
                if ((!open_nodes_check_map[node_index] && cost_to_come.find(node_index) == cost_to_come.end())
                        || temp_cost_to_come <  (cost_to_come | node_index)) {
                    // Update various costs and nodes
                    parent_nodes[node_index] = current_node_index;
                    cost_to_come[node_index] = temp_cost_to_come;
                    final_cost[node_index] = temp_cost_to_come + CostToGo(x, y, epsilon);
                    open_nodes.push_back(Node(x, y, final_cost[node_index]));

                    // Remove node from closed nodes if it exists in there
                    if (closed_nodes[node_index] != kNoParent || closed_nodes.find(node_index) != closed_nodes.end()) {
                        closed_nodes[node_index] = kNoParent;
                    }
                }
            }
            // Get node with minimum cost on top
            std::make_heap(open_nodes.begin(), open_nodes.end(), CompareTotalCost());
        }
    }

    // Log that path was not found
    if (!path_found) {
        logger.Log("Path to goal NOT FOUND!", kDebug);
    }

    return path_found;
}

bool PathFinder::AraStar(float epsilon) {
    logger.Log("Method is UNDER DEVELOPMENT! Use some other method.", kInfo);
    // Initialize cost of goal node
    cost_to_come[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])] = INFINITY;

    uint32_t output_count = 0;

    // Initialize open nodes
    final_cost[RavelIndex(robot_start_pos[0], robot_start_pos[1])] *= epsilon;
    open_nodes.push_back(Node(robot_start_pos[0], robot_start_pos[1],
                            final_cost[RavelIndex(robot_start_pos[0], robot_start_pos[1])],
                            cost_to_come[RavelIndex(robot_start_pos[0], robot_start_pos[1])] +
                                CostToGo(robot_start_pos[0], robot_start_pos[1])));
    
    // Find goal node
    ImprovePath(epsilon);
    closed_nodes[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])] = parent_nodes[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])];

    // Publish current sub-optimal solution
    float bound = static_cast<float>(cost_to_come[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])]/GetMinCost());
    float temp_epsilon = epsilon < bound ? epsilon : bound;
    GeneratePathList(closed_nodes, ++output_count);

    while (temp_epsilon > 1) {
        epsilon = temp_epsilon - 0.01;

        // Move nodes from inconsistent set to open set
        // Update f-value of each node in the inconsisten set
        for (auto it = std::begin(incons_nodes); it != std::end(incons_nodes); ++it) {
            Node node = *it;
            node.final_cost = cost_to_come[RavelIndex(node.x, node.y)] + CostToGo(node.x, node.y, epsilon);
            open_nodes.push_back(node);
        }

        // Empty inconsistent and closed sets
        incons_nodes.clear();
        closed_nodes.clear();

        // Try to improve path
        ImprovePath(epsilon);
        closed_nodes[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])] = parent_nodes[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])];

        // Publish current sub-optimal solution
        bound = static_cast<float>(cost_to_come[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])]/GetMinCost());
        temp_epsilon = epsilon < bound ? epsilon : bound;
        GeneratePathList(closed_nodes, ++output_count);
        break;
    }

    return true;
}

bool PathFinder::AnaStar() {
    logger.Log("Method is UNDER DEVELOPMENT! Use some other method.", kInfo);
    return false;
}

bool PathFinder::IsNodeValid(uint16_t pos_x, uint16_t pos_y) {
    pos_y = robot_world_size[1] - pos_y;
    // Boundary and obstacle space check
    if (pos_x <= 0 || pos_x > robot_world_size[0] || pos_y <= 0 || pos_y > robot_world_size[1]) {
        return false;
    } else if (static_cast<uint16_t>(robot_world.at<uchar>(pos_y, pos_x)) == 0) {
        return false;
    }

    return true;
}

void PathFinder::ImprovePath(float epsilon) {
    // Get node with minimum f-value on top
    std::make_heap(open_nodes.begin(), open_nodes.end(), CompareTotalCost());

    while (cost_to_come[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])] > open_nodes.front().final_cost) {
        Node current_node = open_nodes.front();
        open_nodes.erase(open_nodes.begin());
        uint32_t current_node_index = RavelIndex(current_node.x, current_node.y);

        // Add extracted node to the list of closed nodes
        // FIXME: Fix Updating of parent nodes and backtracking
        if (!(current_node.x == robot_start_pos[0] && current_node.y == robot_start_pos[1])) {
            closed_nodes[current_node_index] = parent_nodes[current_node_index];
        } else {
            closed_nodes[RavelIndex(robot_start_pos[0], robot_start_pos[1])] = kStartParent;
        }

        // Get neighbors to the current node
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_node.x, i);
            uint16_t y = actions.GetNextCoord(current_node.y, i, 'y');

            double temp_cost_to_come = CostToCome(cost_to_come[current_node_index], i);
            uint32_t node_index = RavelIndex(x, y);

            if (IsNodeValid(x, y) && (cost_to_come | node_index) > temp_cost_to_come) {
                // Update various costs and nodes
                cost_to_come[node_index] = temp_cost_to_come;
                parent_nodes[node_index] = current_node_index;
                // Add node to open nodes if it doesn't exist in closed nodes
                if ((closed_nodes % node_index) == kNoParent) {
                    final_cost[node_index] = temp_cost_to_come + CostToGo(x, y, epsilon);
                    open_nodes.push_back(Node(x, y, final_cost[node_index], temp_cost_to_come + CostToGo(x, y)));
                    // logger.Log("New operator is working!", kDebug);
                } else {
                    incons_nodes.push_back(Node(x, y, final_cost[node_index], temp_cost_to_come + CostToGo(x, y)));
                }
            }
        }

        // Get node with minimum cost on top
        std::make_heap(open_nodes.begin(), open_nodes.end(), CompareTotalCost());
    }

    logger.Log("Goal found!", kDebug);
}

double PathFinder::GetMinCost() {
    std::make_heap(open_nodes.begin(), open_nodes.end(), ComparePsuedoCost());

    if (incons_nodes.empty()) {
        return open_nodes.front().psuedo_cost;
    }

    std::make_heap(incons_nodes.begin(), incons_nodes.end(), ComparePsuedoCost());

    return (open_nodes.front().psuedo_cost < incons_nodes.front().psuedo_cost ?
            open_nodes.front().psuedo_cost : incons_nodes.front().psuedo_cost);
}

double PathFinder::CostToCome(double parent_node_cost, uint8_t action) {
    if (action < kUpRight) {
        return parent_node_cost + 1;
    }

    return parent_node_cost + sqrt(2);
}

double PathFinder::CostToGo(uint16_t pos_x, uint16_t pos_y, float epsilon) {
    return epsilon * sqrt(pow(robot_goal_pos[0] - pos_x, 2) + pow(robot_goal_pos[1] - pos_y, 2));
}

uint32_t PathFinder::RavelIndex(uint16_t pos_x, uint16_t pos_y) {
    return (pos_y * robot_world_size[0]) + pos_x;
}

std::pair<uint16_t, uint16_t> PathFinder::UnravelIndex(uint32_t identifier) {
    return std::make_pair(static_cast<int>(identifier%robot_world_size[0]),
                    static_cast<int>(identifier/robot_world_size[0]));
}

PathFinder::~PathFinder() {
}
