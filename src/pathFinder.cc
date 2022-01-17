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

// C headers
#include <math.h>
// C++ headers
#include <iostream>
#include <fstream>
#include <chrono>
// Other headers
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

void PathFinder::GeneratePathList(Node* goal, uint32_t list_index) {
    std::string file_name = "path/pathList_" + std::to_string(list_index) + "_.txt";
    std::ofstream path_list;
    path_list.open(file_name, std::ios::out | std::ios::trunc);

    // Add goal node into the list of path nodes
    auto last_coords = goal->GetCoordinates();
    path_list << last_coords[0] << ", " << last_coords[1] << std::endl;
    Node *last_node = goal->GetParent();

    // Backtrack the start node
    while (last_node != NULL) {
        last_coords = last_node->GetCoordinates();
        path_list << last_coords[0] << ", " << last_coords[1] << std::endl;
        last_node = last_node->GetParent();
    }

    path_list.close();
    logger.Log("Path list GENERATED!", kDebug);
}

bool PathFinder::Astar() {
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // Add parent map
    std::map<uint32_t, int8_t> parent_map;
    parent_map[RavelIndex(robot_start_pos[0], robot_start_pos[1])] = kStartParent;

    // Initialize open nodes
    Node *current_node;
    int16_t current_node_index;
    open_nodes.AddNode(new Node(robot_start_pos[0], robot_start_pos[1], 0,
                        CostToGo(robot_start_pos[0], robot_start_pos[1]), NULL));

    // Try finding path to goal until the queue goes empty
    while (!open_nodes.IsEmpty()) {
        // Extract the node with minimum cost and add it to the closed nodes list
        current_node_index = open_nodes.GetTopNode();
        current_node = open_nodes.GetNode(current_node_index);
        open_nodes.DeleteNode(current_node_index);
        closed_nodes.AddNode(current_node);

        auto current_coords = current_node->GetCoordinates();

        // Exit if goal is found
        if (current_coords[0] == robot_goal_pos[0] && current_coords[1] == robot_goal_pos[1]) {
            logger.Log("Path to goal FOUND!", kDebug);
            GeneratePathList(current_node, 1);
            
            // Record time
            auto stop = std::chrono::high_resolution_clock::now();
            std::cout << "Time taken in ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << std::endl;
            return true;
        }

        // Generate child nodes
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_coords[0], i);
            uint16_t y = actions.GetNextCoord(current_coords[1], i, 'y');

            // Make sure node is not in obstacle space and it is better then the previous one
            if (IsNodeValid(x, y) && parent_map.find(RavelIndex(x, y)) == parent_map.end()) {
                // Update parent and cost-to-come
                parent_map[RavelIndex(x, y)] = 1;
                double cost2come = CostToCome(current_node->GetCostToCome(), i);
                // Add nodes to open list
                open_nodes.AddNode(new Node(x, y, cost2come, cost2come + CostToGo(x, y), current_node));
            }
        }
    }

    logger.Log("Path to goal NOT FOUND!", kDebug);
    return false;
}

bool PathFinder::AtaStar(float epsilon) {
    // Initialize bound, inflation factor, and output counter
    double bound = INFINITY;
    uint32_t output_count = 0;
    bool path_found = false;

    // Add cost-to-come map
    std::map<uint32_t, int8_t> cost2come;
    cost2come[RavelIndex(robot_start_pos[0], robot_start_pos[1])] = 0;

    // Initialize open nodes
    Node *current_node;
    int16_t current_node_index;
    open_nodes.AddNode(new Node(robot_start_pos[0], robot_start_pos[1], 0,
                            CostToGo(robot_start_pos[0], robot_start_pos[1], epsilon),
                            NULL));

    // Try finding path to goal until the queue goes empty
    while (!open_nodes.IsEmpty()) {
        // Extract the node with minimum cost and add it to the closed nodes' list
        current_node_index = open_nodes.GetTopNode();
        current_node = open_nodes.GetNode(current_node_index);
        open_nodes.DeleteNode(current_node_index);
        closed_nodes.AddNode(current_node);

        auto current_coords = current_node->GetCoordinates();

        // Get path if goal is found
        if (current_coords[0] == robot_goal_pos[0] && current_coords[1] == robot_goal_pos[1]) {
            path_found = true;
            logger.Log("Path to goal FOUND!", kDebug);

            // Update the bound and generate path text file
            bound = current_node->GetCostToCome() + CostToGo(current_coords[0], current_coords[1]);
            GeneratePathList(current_node, ++output_count);

            // Prune open nodes list if it is not empty
            if (!open_nodes.IsEmpty()) {
                logger.Log("Pruning open nodes list...", kInfo);
                auto &open_nodes_list = open_nodes.GetList();
                Node *node;
                for (auto it = open_nodes_list.begin(); it != open_nodes_list.end(); ++it) {
                    node = *it;
                    auto node_coords = node->GetCoordinates();
                    if (node->GetCostToCome() + CostToGo(node_coords[0], node_coords[1]) >= bound) {
                        open_nodes_list.erase(it);
                    }
                    if (open_nodes_list.empty()) {
                        break;
                    }
                }
            } else {
                logger.Log("Open nodes list is EMPTY!", kWarn);
                return path_found;
            }
        }

        // Get neighbors to the current node
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_coords[0], i);
            uint16_t y = actions.GetNextCoord(current_coords[1], i, 'y');

            double temp_cost2come = CostToCome(current_node->GetCostToCome(), i);

            // Make sure node is not in obstacle space and it obeys the bound
            if (IsNodeValid(x, y) && temp_cost2come + CostToGo(x, y) < bound) {
                uint32_t node_index = RavelIndex(x, y);
                int16_t in_closed_index = closed_nodes.FindNode(x, y);
                double new_cost2come = (cost2come.find(node_index) == cost2come.end() ? __DBL_MAX__ : cost2come[node_index]);
                // Make sure the node is neither in open nodes nor in closed nodes
                if ((in_closed_index == NOT_IN_LIST && open_nodes.FindNode(x, y) == NOT_IN_LIST)
                    || temp_cost2come < new_cost2come) {
                    // Update various costs and nodes
                    cost2come[node_index] = temp_cost2come;
                    open_nodes.AddNode(new Node(x, y, temp_cost2come,
                                            temp_cost2come + CostToGo(x, y, epsilon),
                                            current_node));

                    // Remove node from closed nodes if it exists in there
                    if (in_closed_index != NOT_IN_LIST) {
                        closed_nodes.DeleteNode(in_closed_index);
                    }
                }
            }
        }
    }

    // Log that path was not found
    if (!path_found) {
        logger.Log("Path to goal NOT FOUND!", kDebug);
    }

    return path_found;
}

bool PathFinder::AraStar(float epsilon) {
    uint32_t output_count = 0;
    Node *temp_node;

    // Initialize open nodes
    open_nodes.AddNode(new Node(robot_start_pos[0], robot_start_pos[1], 0,
                            CostToGo(robot_start_pos[0], robot_start_pos[1], epsilon),
                            NULL));

    // Find goal node
    Node *goal_node = new Node(robot_goal_pos[0], robot_goal_pos[1], __DBL_MAX__, __DBL_MAX__, NULL);
    goal_node = ImprovePath(goal_node, epsilon);

    // Publish current best solution
    GeneratePathList(goal_node, ++output_count);

    while (epsilon > 1) {
        epsilon--;

        // Move nodes from inconsistent set to open set
        auto &incons_nodes_list = incons_nodes.GetList();
        for (auto it = incons_nodes_list.begin(); it != incons_nodes_list.end(); ++it) {
            temp_node = *it;
            open_nodes.AddNode(new Node(temp_node->GetCoordinates()[0], temp_node->GetCoordinates()[1], temp_node->GetCostToCome(),
                                    temp_node->GetFinalCost(), temp_node->GetParent()));
        }
        incons_nodes.ClearList();

        // Empty closed sets
        closed_nodes.ClearList();

        // Try to improve path
        goal_node = ImprovePath(goal_node, epsilon);

        // Publish current best solution
        GeneratePathList(goal_node, ++output_count);
    }

    return true;
}

bool PathFinder::AnaStar() {
    logger.Log("Method is UNDER DEVELOPMENT! Use some other method.", kInfo);
    return false;
}

Node* PathFinder::ImprovePath(Node* goal, float epsilon) {
    Node *current_node, *temp;
    int16_t current_node_index;
    while(!open_nodes.IsEmpty()) {
        // Get node with minimum f-value on top
        current_node_index = open_nodes.GetTopNode();
        current_node = open_nodes.GetNode(current_node_index);
        open_nodes.DeleteNode(current_node_index);
        visited_nodes.AddNode(current_node);
        closed_nodes.AddNode(current_node);

        auto current_coords = current_node->GetCoordinates();

        // Exit if goal is found
        if (current_coords[0] == robot_goal_pos[0] && current_coords[1] == robot_goal_pos[1]) {
            goal->SetParent(current_node->GetParent());
            goal->SetCostToCome(current_node->GetCostToCome());
            goal->SetFinalCost(current_node->GetFinalCost());
            logger.Log("Goal found!", kDebug);
            return goal;
        }

        // Get neighbors to the current node
        for (int i = 0; i < actions.kMaxNumActions; ++i) {
            uint16_t x = actions.GetNextCoord(current_coords[0], i);
            uint16_t y = actions.GetNextCoord(current_coords[1], i, 'y');

            if (IsNodeValid(x, y)) {
                double cost2come = CostToCome(current_node->GetCostToCome(), i);
                double final_cost = cost2come + CostToGo(x, y, epsilon);

                // Add newly discovered node, else update prior node
                if (visited_nodes.FindNode(x, y) == NOT_IN_LIST) {
                    open_nodes.AddNode(new Node(x, y, cost2come, final_cost, current_node));
                    // logger.Log("I'm in visited_nodes if", kDebug);
                } else {
                    int16_t index = closed_nodes.FindNode(x, y);
                    if (index != NOT_IN_LIST) {
                        temp = closed_nodes.GetNode(index);
                        if (cost2come < temp->GetCostToCome()) {
                            temp->SetCostToCome(cost2come);
                            temp->SetParent(current_node);
                            temp->SetFinalCost(final_cost);
                            incons_nodes.AddNode(temp);
                        }
                    }
                }
            }
        }
    }
    logger.Log("Goal found!", kFatal);
    return NULL;
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
