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

#include <opencv2/opencv.hpp>
#include "actions/actions.h"

using namespace std;

class PathFinder {
    private:
        uint16_t robot_start_pos[2], robot_goal_pos[2];
        uint16_t robot_world_size[2];
        cv::Mat robot_world;
        struct MinHeapComparator {
            bool operator()(vector<float> const& a, vector<float> const& b) const{
                // sanity checks
                assert(a.size() == 3);
                assert(b.size() == 3);

                // reverse sort puts the lowest value at the top    
                return a[2] > b[2];
            }
        };

        void PrintVector(vector<float> vec);

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
        PathFinder(uint16_t start_x, uint16_t start_y, uint16_t goal_x, uint16_t goal_y, string robot_world_loc);

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
         * @brief Finds a path from start to goal if it exists
         * @param none
         * @return true if path if found
         */
        bool FindPathToGoal();

        /**
         * @brief Generates a text file listing the nodes in the path
         * @param none
         * @return true if text file generate is a success
         */
        bool GeneratePathList();
};


#endif  // INCLUDE_PATHFINDER_PATHFINDER_H_
