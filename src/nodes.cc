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
 * @file nodes.cc
 * @author Umang Rastogi
 * @brief Implements the classes to work with various nodes in path planner
 */

#include <algorithm>
#include "structures/nodes.h"

// <---Methods of Node class--->
Node::Node(uint16_t x, uint16_t y, double cost2come, double cost2go, Node* parent_node) {
    coordinates.push_back(x);
    coordinates.push_back(y);
    parent = parent_node;
    g_value = cost2come;
    f_value = cost2go;
}

Node::~Node() {
}

std::vector<uint16_t> Node::GetCoordinates() {
    return coordinates;
}

Node* Node::GetParent() {
    return parent;
}

void Node::SetParent(Node* p) {
    parent = p;
}

double Node::GetCostToGo() {
    return f_value;
}

void Node::SetCostToGo(double cost) {
    f_value = cost;
}

double Node::GetCostToCome() {
    return g_value;
}

void Node::SetCostToCome(double cost) {
    g_value = cost;
}

// <---Methods of NodeList class--->
void NodeList::AddNode(Node* node) {
    nodes.push_back(node);
}

Node* NodeList::GetTopNode() {
    std::sort(nodes.begin(), nodes.end());
    return nodes.front();
}

void NodeList::Pop() {
    nodes.erase(nodes.begin());
}

Node* NodeList::GetLastNode() {
    return nodes.back();
}

Node* NodeList::GetNode(int index) {
    return nodes[index];
}

void NodeList::DeleteNode(int index) {
    nodes.erase(nodes.begin() + index);
    return;
}

bool NodeList::IsEmpty() {
    return nodes.empty();
}

int16_t NodeList::FindNode(Node* node) {
    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        if (node == *it) {
            return it - nodes.begin();
        }
    }
    return -1;
}

std::vector<Node*> NodeList::GetList() {
    return nodes;
}
