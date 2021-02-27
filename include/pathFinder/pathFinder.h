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
        uint16_t start_x, start_y, goal_x, goal_y;
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
         * @param xs x-coordinate of robot's start postion
         * @param ys y-coordinate of robot's start postion
         * @param xg x-coordinate of robot's goal postion
         * @param yg y-coordinate of robot's goal postion
         * @param robot_world_loc Location of robot's world image
         * @return none
         */
        PathFinder(uint16_t xs, uint16_t ys, uint16_t xg, uint16_t yg, string robot_world_loc);

        /**
         * @brief Destructor for the class
         * @param none
         * @return none
         */
        ~PathFinder();

        int8_t FindPathToGoal();

        void GeneratePathList();
};


#endif  // INCLUDE_PATHFINDER_PATHFINDER_H_
