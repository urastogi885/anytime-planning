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
 * @file explorer.cpp
 * @author Umang Rastogi
 * @brief Main file to run the entire project
 */

#include <iostream>
#include "actions.hpp"

/**
 * @brief Main entry point of the project
 */
int main(int argc, char ** argv) {
    Actions actions;
    for (uint8_t i = 0; i < actions.kMaxNumActions; i++) {
        printf("Next child node: (%d, %d)\n", actions.GetNextCoord(50, i), actions.GetNextCoord(30, i, 'y'));
    }
    
    if (actions.GetNextCoord(50, 10) < 0) {
        printf("Invalid action for the robot!\n");
        return -1;
    }

    return 0;
}