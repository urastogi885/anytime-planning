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

// C headers
#include <math.h>
// C++ headers
#include <cstdint>
#include <unordered_map>

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

enum Methods {
    kAstar = 0,     // Use A* method to find path
    kAtaStar,       // Use Anytime A* method to find path
    kAraStar,       // Use Anytime Repairing A* method to find path
    kAnaStar        // Use Anytime Nonparametric A* method to find path
};

enum ExitCodes {
    kSuccess = 0,       // Path found from start to goal
    kInsuffArgs,        // Insufficient arguments provided
    kObtsacleSpace,     // Start or goal lie in obatcle space
    kPathNotExist       // Path does exist from start to goal
};

struct Node {
    uint16_t x, y;
    double final_cost;

    Node(uint16_t x, uint16_t y, double cost)
        : x(x), y(y), final_cost(cost) {
    }
};

struct CompareTotalCost {
    bool operator()(Node const& a, Node const& b) const{
        // reverse sort to put the lowest value at the top
        return a.final_cost > b.final_cost;
    }
};

// Return infinity as default value for unordered maps
template<typename K, typename M>
M& operator|(std::unordered_map<K, M>& umap, const K& key) {
    static M defval{INFINITY};

    if (!umap.count(key)) {
        return defval;
    }

    return umap[key];
}

#endif  //  INCLUDE_STRUCTURES_STRUCTURES_H_
