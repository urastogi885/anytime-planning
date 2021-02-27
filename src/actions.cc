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
 * @file actions.cc
 * @author Umang Rastogi
 * @brief Implements the Actions' class to return coordinates of the next node
 */

#include "actions/actions.h"

Actions::Actions() {
    coordType = 'x';
    coordinate = 0;
}

int Actions::GetNextCoord(uint16_t prevCoord, uint8_t action, char type) {
    coordinate = prevCoord;
    coordType = type;
    switch (action) {
    case kUp:
        return GoUp();

    case kUpRight:
        return GoUpRight();

    case kRight:
        return GoRight();

    case kDownRight:
        return GoDownRight();

    case kDown:
        return GoDown();

    case kDownLeft:
        return GoDownLeft();

    case kLeft:
        return GoLeft();

    case kUpLeft:
        return GoUpLeft();

    default:
        return -1;
    }
}

uint16_t Actions::GoUp() {
    return (coordType == 'x' ? coordinate : ++coordinate);
}

uint16_t Actions::GoUpRight() {
    return ++coordinate;
}

uint16_t Actions::GoRight() {
    return (coordType == 'x' ? ++coordinate : coordinate);
}

uint16_t Actions::GoDownRight() {
    return (coordType == 'x' ? --coordinate : ++coordinate);
}

uint16_t Actions::GoDown() {
    return (coordType == 'x' ? coordinate : --coordinate);
}

uint16_t Actions::GoDownLeft() {
    return --coordinate;
}

uint16_t Actions::GoLeft() {
    return (coordType == 'x' ? --coordinate : coordinate);
}

uint16_t Actions::GoUpLeft() {
    return (coordType == 'x' ? --coordinate : ++coordinate);
}

Actions::~Actions() {
}
