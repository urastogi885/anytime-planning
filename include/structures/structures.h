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
 * @file structures.h
 * @author Umang Rastogi
 * @brief Defines various useful data structures for the project
 */

#ifndef INCLUDE_STRUCTURES_STRUCTURES_H_
#define INCLUDE_STRUCTURES_STRUCTURES_H_

#include <cstdint>
#include <vector>

enum Action {
    kUp = 0,
    kDown,
    kRight,
    kLeft,
    kUpRight,
    kUpLeft,
    kDownRight,
    kDownLeft
};

enum LogLevel {
    kInfo = 0,
    kWarn,
    kDebug,
    kFatal
};

enum ParentIdentifier {
    kNoParent = 0,  // Node does not have a parent yet
    kParent,        // Node has a parent
    kStartParent    // Node is the start node
};

struct Node {
    uint16_t x, y;
    float cost_to_come, total_cost;

    Node(uint16_t x, uint16_t y, float cost[2])
        : x(x), y(y), cost_to_come(cost[0]), total_cost(cost[1])
    {
    }
};

struct CompareCostToCome {
    bool operator()(Node const& a, Node const& b) const{

        // reverse sort puts the lowest value at the top    
        return a.total_cost > b.total_cost;
    }
};

#endif  //  INCLUDE_STRUCTURES_STRUCTURES_H_
