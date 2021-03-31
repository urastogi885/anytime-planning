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
#include <chrono>
#include "pathFinder/pathFinder.h"

PathFinder::PathFinder(uint16_t start_x, uint16_t start_y, uint16_t goal_x,
                uint16_t goal_y, const char * robot_world_loc) {
    // Store start and goal position of the robot
    robot_start_pos[0] = goal_x;
    robot_start_pos[1] = goal_y;
    robot_goal_pos[0] = start_x;
    robot_goal_pos[1] = start_y;
    // Read the image and store its size
    robot_world = cv::imread(robot_world_loc, CV_LOAD_IMAGE_GRAYSCALE);
    robot_world_size[1] = robot_world.rows;
    robot_world_size[0] = robot_world.cols;
    // Initialize parent nodes and cost maps
    uint32_t start_node_index = RavelIndex(robot_start_pos[0], robot_start_pos[1]);
    cost_to_come[start_node_index] = 0;
    final_cost[start_node_index] = CostToGo(robot_start_pos[0], robot_start_pos[1]);
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

void PathFinder::GeneratePathList(uint32_t list_index) {
    std::string file_name = "path/pathList_" + std::to_string(list_index) + "_.txt";
    std::ofstream path_list;
    path_list.open(file_name, std::ios::out | std::ios::trunc);

    // Add goal node into the list of path nodes
    Node last_node = closed_nodes.back();
    path_list << last_node.x << ", " << last_node.y << std::endl;

    // Backtrack the start node
    while (last_node.parent != kStartParent) {
        for (auto it = closed_nodes.begin(); it != closed_nodes.end(); ++it) {
            Node node = *it;
            if (RavelIndex(node.x, node.y) == last_node.parent) {
                last_node = node;
                path_list << last_node.x << ", " << last_node.y << std::endl;
                break;
            }
        }
    }

    path_list.close();
    logger.Log("Path list GENERATED!", kDebug);

    return;
}

bool PathFinder::Astar() {
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();
    // Initialize open nodes
    open_nodes.push_back(Node(robot_start_pos[0], robot_start_pos[1],
                        final_cost[RavelIndex(robot_start_pos[0], robot_start_pos[1])],
                        kStartParent));
    // Try finding path to goal until the queue goes empty
    while (!open_nodes.empty()) {
        // Extract the node with minimum cost and add it to the closed nodes list
        Node current_node = open_nodes.front();
        open_nodes.erase(open_nodes.begin());
        closed_nodes.push_back(current_node);

        uint32_t current_node_index = RavelIndex(current_node.x, current_node.y);

        // Exit if goal is found
        if (current_node.x == robot_goal_pos[0] && current_node.y == robot_goal_pos[1]) {
            logger.Log("Path to goal FOUND!", kDebug);
            GeneratePathList(1);
            
            // Record time
            auto stop = std::chrono::high_resolution_clock::now();
            std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << std::endl;
            return true;
        }

        // Generate child nodes
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_node.x, i);
            uint16_t y = actions.GetNextCoord(current_node.y, i, 'y');

            uint32_t node_index = RavelIndex(x, y);
            double temp_cost_to_come = CostToCome((cost_to_come | current_node_index), i);

            // Make sure node is not in obstacle space and it is better then the previous one
            if (IsNodeValid(x, y) && temp_cost_to_come <  (cost_to_come | node_index)) {
                // Update costs
                cost_to_come[node_index] = temp_cost_to_come;
                final_cost[node_index] = temp_cost_to_come + CostToGo(x, y);
                // If new, add to open nodes otherwise update parent
                if (!FindNodeInList(open_nodes, x, y)) {
                    open_nodes.push_back(Node(x, y, final_cost[node_index], current_node_index));
                } else {
                    for (auto i = 0; i < open_nodes.size(); ++i) {
                        if(open_nodes[i].x == x && open_nodes[i].y == y) {
                            open_nodes[i].parent = current_node_index;
                        }
                    }
                }
            }
        }
        // Get node with minimum cost on top
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
                            final_cost[RavelIndex(robot_start_pos[0], robot_start_pos[1])],
                            kStartParent));

    // Try finding path to goal until the queue goes empty
    while (!open_nodes.empty()) {
        // Extract the node with minimum cost and add it to the closed nodes' list
        Node current_node = open_nodes.front();
        open_nodes.erase(open_nodes.begin());
        closed_nodes.push_back(current_node);

        uint32_t current_node_index = RavelIndex(current_node.x, current_node.y);

        // Get path if goal is found
        if (current_node.x == robot_goal_pos[0] && current_node.y == robot_goal_pos[1]) {
            path_found = true;
            logger.Log("Path to goal FOUND!", kDebug);

            // Update the bound and generate path text file
            bound = cost_to_come[current_node_index] + CostToGo(current_node.x, current_node.y);
            GeneratePathList(++output_count);

            // Prune open nodes list if it is not empty
            if (!open_nodes.empty()) {
                logger.Log("Pruning open nodes list...", kInfo);
                for (auto it = open_nodes.begin(); it != open_nodes.end(); ++it) {
                    Node node = *it;
                    if (cost_to_come[RavelIndex(node.x, node.y)] + CostToGo(node.x, node.y) >= bound) {
                        open_nodes.erase(it);
                    }
                }
            } else {
                logger.Log("Open nodes list is EMPTY!", kWarn);
                return path_found;
            }
        }

        // Get neighbors to the current node
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_node.x, i);
            uint16_t y = actions.GetNextCoord(current_node.y, i, 'y');

            double temp_cost_to_come = CostToCome(cost_to_come[current_node_index], i);

            // Make sure node is not in obstacle space and it obeys the bound
            if (IsNodeValid(x, y) && temp_cost_to_come + CostToGo(x, y) < bound) {
                uint32_t node_index = RavelIndex(x, y);
                int16_t in_closed_index = FindNodeInList(closed_nodes, x, y);
                // Make sure the node is neither in open nodes nor in closed nodes
                if ((in_closed_index == kNotInList && FindNodeInList(open_nodes, x, y) == kNotInList)
                    || temp_cost_to_come <  (cost_to_come | node_index)) {
                    // Update various costs and nodes
                    cost_to_come[node_index] = temp_cost_to_come;
                    final_cost[node_index] = cost_to_come[node_index] + CostToGo(x, y, epsilon);
                    open_nodes.push_back(Node(x, y, final_cost[node_index], current_node_index));

                    // Remove node from closed nodes if it exists in there
                    if (in_closed_index != kNotInList) {
                        closed_nodes.erase(closed_nodes.begin() + in_closed_index);
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
                            kStartParent,
                            cost_to_come[RavelIndex(robot_start_pos[0], robot_start_pos[1])] +
                                CostToGo(robot_start_pos[0], robot_start_pos[1])));
    
    // Find goal node
    ImprovePath(epsilon);
    // closed_nodes.push_back()[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])] = parent_nodes[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])];

    // Publish current sub-optimal solution
    float bound = static_cast<float>(cost_to_come[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])]/GetMinCost());
    float temp_epsilon = epsilon < bound ? epsilon : bound;
    GeneratePathList(++output_count);

    while (temp_epsilon > 1) {
        epsilon = temp_epsilon - 0.01;

        // Move nodes from inconsistent set to open set
        for (auto it = incons_nodes.begin(); it != incons_nodes.end(); ++it) {
            Node node = *it;
            open_nodes.push_back(node);
        }
        incons_nodes.clear();

        // Update f-value of each node in the open nodes set
        for (auto i = 0; i < open_nodes.size(); ++i) {
            Node node = open_nodes[i];
            open_nodes[i].final_cost = cost_to_come[RavelIndex(node.x, node.y)] + CostToGo(node.x, node.y, epsilon);
        }

        // Empty closed sets
        closed_nodes.clear();

        // Try to improve path
        ImprovePath(epsilon);
        closed_nodes.push_back(open_nodes.front());
        // closed_nodes[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])] = parent_nodes[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])];

        // Publish current sub-optimal solution
        bound = static_cast<float>(cost_to_come[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])]/GetMinCost());
        temp_epsilon = epsilon < bound ? epsilon : bound;
        GeneratePathList(++output_count);
    }

    return true;
}

bool PathFinder::AnaStar() {
    logger.Log("Method is UNDER DEVELOPMENT! Use some other method.", kInfo);
    return false;
}

void PathFinder::ImprovePath(float epsilon) {
    // Get node with minimum f-value on top
    std::make_heap(open_nodes.begin(), open_nodes.end(), CompareTotalCost());

    while (cost_to_come[RavelIndex(robot_goal_pos[0], robot_goal_pos[1])] > open_nodes.front().final_cost) {
        Node current_node = open_nodes.front();
        open_nodes.erase(open_nodes.begin());
        closed_nodes.push_back(current_node);

        uint32_t current_node_index = RavelIndex(current_node.x, current_node.y);

        // Add extracted node to the list of closed nodes
        // FIXME: Fix Updating of parent nodes and backtracking
        // if (!(current_node.x == robot_start_pos[0] && current_node.y == robot_start_pos[1])) {
        //     closed_nodes[current_node_index] = parent_nodes[current_node_index];
        // } else {
        //     closed_nodes[RavelIndex(robot_start_pos[0], robot_start_pos[1])] = kStartParent;
        // }

        // Get neighbors to the current node
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_node.x, i);
            uint16_t y = actions.GetNextCoord(current_node.y, i, 'y');

            double temp_cost_to_come = CostToCome(cost_to_come[current_node_index], i);
            uint32_t node_index = RavelIndex(x, y);

            if (IsNodeValid(x, y) && (cost_to_come | node_index) > temp_cost_to_come) {
                // Update various costs and nodes
                cost_to_come[node_index] = temp_cost_to_come;
                // Add node to open nodes if it doesn't exist in closed nodes
                if (!FindNodeInList(closed_nodes, x, y)) {
                    final_cost[node_index] = cost_to_come[node_index] + CostToGo(x, y, epsilon);
                    open_nodes.push_back(Node(x, y, final_cost[node_index], current_node_index, temp_cost_to_come + CostToGo(x, y)));
                    // logger.Log("New operator is working!", kDebug);
                } else {
                    incons_nodes.push_back(Node(x, y, final_cost[node_index], current_node_index, temp_cost_to_come + CostToGo(x, y)));
                }
            }
        }

        // Get node with minimum cost on top
        std::make_heap(open_nodes.begin(), open_nodes.end(), CompareTotalCost());
    }

    logger.Log("Goal found!", kDebug);
}

int16_t PathFinder::FindNodeInList(std::vector<Node> &node_list, uint16_t x, uint16_t y) {
    for (auto it = node_list.begin(); it != node_list.end(); ++it) {
        Node node = *it;
        if (node.x == x && node.y == y) {
            return it - node_list.begin();
        }
    }
    return kNotInList;
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

int32_t PathFinder::RavelIndex(uint16_t pos_x, uint16_t pos_y) {
    return (pos_y * robot_world_size[0]) + pos_x;
}

std::pair<uint16_t, uint16_t> PathFinder::UnravelIndex(int32_t identifier) {
    return std::make_pair(static_cast<uint16_t>(identifier%robot_world_size[0]),
                    static_cast<uint16_t>(identifier/robot_world_size[0]));
}

PathFinder::~PathFinder() {
}
