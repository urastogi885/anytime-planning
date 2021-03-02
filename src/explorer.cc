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
 * @file explorer.cc
 * @author Umang Rastogi
 * @brief Main file to run the entire project
 */

#include "errorLogger/errorLogger.h"
#include "pathFinder/pathFinder.h"

/**
 * @brief Main entry point of the project
 */
int main(int argc, char ** argv) {
    if (argc != 6) {
        ErrorLogger(kInfo).Log("Insufficient arguments provided!", kFatal);
        return -1;
    }
    PathFinder path_finder = PathFinder(atoi(argv[1]), atoi(argv[2]),
                                atoi(argv[3]), atoi(argv[4]), std::string(argv[5]));

    if (path_finder.FindPathToGoal()) {
        path_finder.GeneratePathList();
    }

    return 0;
}
