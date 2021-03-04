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
 * @file pathFinder.h
 * @author Umang Rastogi
 * @brief Defines the PathFinder's class to return coordinates of the next node
 */

#ifndef INCLUDE_PATHFINDER_PATHFINDER_H_
#define INCLUDE_PATHFINDER_PATHFINDER_H_

// C headers
#include <math.h>
// C++ headers
#include <cstdint>
#include <utility>
#include <vector>
#include <queue>
#include <unordered_map>
#include <opencv2/opencv.hpp>
// Other headers
#include "actions/actions.h"
#include "structures/structures.h"
#include "consoleLogger/consoleLogger.h"

class PathFinder {
    private:
        // Constants
        const int8_t kNoParent = -1;
        const int8_t kStartParent = -2;
        const char kPathListFileName[13] = "pathList.txt";
        // Robot world
        uint16_t robot_start_pos[2], robot_goal_pos[2];
        uint16_t robot_world_size[2];
        cv::Mat robot_world;
        // Nodes
        std::unordered_map<uint32_t, int64_t> parent_nodes;
        std::unordered_map<uint32_t, double> cost_to_come;
        std::unordered_map<uint32_t, double> final_cost;
        std::priority_queue<Node, std::vector<Node>, CompareTotalCost> open_nodes;
        std::unordered_map<uint32_t, bool> open_nodes_check_map;
        // Class objects
        ConsoleLogger logger = ConsoleLogger(kInfo);
        Actions actions;

        /**
         * @brief Finds cost to reach a node
         * @param parent_node_cost cost of the node's parent
         * @param action action that yields the node
         * @return cost to reach the node
         */
        double CostToCome(double parent_node_cost, uint8_t action);

        /**
         * @brief Finds an estimate of the cost to reach the goal from a node (heuristic)
         * @param pos_x x-coordinate of the node
         * @param pos_y y-coordinate of the node
         * @param epsilon inflation factor; minimum value = 1
         * @return an estimate of the cost cost to reach the goal
         */
        double CostToGo(uint16_t pos_x, uint16_t pos_y, float epsilon);

        /**
         * @brief Convert an element location into a unique integer
         * @param pos_x x-coordinate of the node
         * @param pos_y y-coordinate of the node
         * @return a unique identifying integer
         */
        uint32_t RavelIndex(uint16_t pos_x, uint16_t pos_y);

        /**
         * @brief Convert an integer into an element location
         * @param identifier x-coordinate of the node
         * @return location of the element
         */
        std::pair<uint16_t, uint16_t> UnravelIndex(uint32_t identifier);

    public:
        /**
         * @brief Constructor for the class
         * @param start_x x-coordinate of robot's start postion
         * @param start_y y-coordinate of robot's start postion
         * @param goal_x x-coordinate of robot's goal postion
         * @param goal_y y-coordinate of robot's goal postion
         * @param robot_world_loc Location of robot's world image
         * @return none
         */
        PathFinder(uint16_t start_x, uint16_t start_y, uint16_t goal_x, uint16_t goal_y, const char * robot_world_loc);

        /**
         * @brief Destructor for the class
         * @param none
         * @return none
         */
        ~PathFinder();

        /**
         * @brief Checks whether a node lies in obstacle space
         * @param pos_x x-coordinate of the node
         * @param pos_y y-coordinate of the node
         * @return true if node does not lie in obstacle space
         */
        bool IsNodeValid(uint16_t pos_x, uint16_t pos_y);

        /**
         * @brief Calls appropriate path finding method
         * @param method Specifies the method being used to find path to goal
         * @return true if path is found
         */
        bool FindPathToGoal(uint8_t method = kAstar);

        /**
         * @brief Generates a text file listing the nodes in the path
         * @param none
         * @return nothing
         */
        void GeneratePathList();

        /**
         * @brief Finds a path from start to goal if it exists using A*
         * @param none
         * @return nothing
         */
        bool Astar();

        /**
         * @brief Finds a path from start to goal if it exists using ATA*
         * @param none
         * @return nothing
         */
        bool AtaStar();

        /**
         * @brief Finds a path from start to goal if it exists using ARA*
         * @param none
         * @return nothing
         */
        bool AraStar();

        /**
         * @brief Finds a path from start to goal if it exists using ANA*
         * @param none
         * @return nothing
         */
        bool AnaStar();
};

#endif  // INCLUDE_PATHFINDER_PATHFINDER_H_
