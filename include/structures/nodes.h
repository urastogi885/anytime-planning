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
 * @file nodes.h
 * @author Umang Rastogi
 * @brief Defines the classes to work with various nodes in path planner
 */

#ifndef INCLUDE_STRUCTURES_NODES_H_
#define INCLUDE_STRUCTURES_NODES_H_

// C++ headers
#include <cstdint>
#include <vector>

class Node {
    std::vector<uint16_t> coordinates;
    Node* parent;
    double g_value;
    double f_value;

    public:
        /**
         * @brief Constructor for the node class
         */
        Node(uint16_t x, uint16_t y, double cost2come, double cost2go, Node* parent_node);

        ~Node();

        std::vector<uint16_t> GetCoordinates();

        Node* GetParent();

        void SetParent(Node* p);

        double GetCostToGo();

        void SetCostToGo(double cost);

        double GetCostToCome();

        void SetCostToCome(double cost);

        bool operator == (const Node& node) const { return (coordinates[0] == node.coordinates[0] && coordinates[1] == node.coordinates[1]); }

        bool operator < (const Node& node) const { return f_value < node.f_value; }
};

class NodeList {
    std::vector<Node*> nodes;

    public:
        void AddNode(Node* node);

        Node* GetTopNode();
        
        void Pop();

        Node* GetLastNode();

        Node* GetNode(int index);

        void DeleteNode(int index);

        bool IsEmpty();

        int16_t FindNode(Node* node);

        std::vector<Node*> GetList();
};

#endif  // INCLUDE_STRUCTURES_NODES_H_