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
 * @file actions.h
 * @author Umang Rastogi
 * @brief Defines the Actions' class to return coordinates of the next node
 */

#ifndef INCLUDE_ACTIONS_ACTIONS_H_
#define INCLUDE_ACTIONS_ACTIONS_H_

// C++ headers
#include <cstdint>

class Actions {
    private:
        uint16_t coordinate;
        char coordType;

        /**
         * @brief Get coordinates after an upward movement
         * @param none
         * @return Coordinate of the next node
         */
        uint16_t GoUp();

        /**
         * @brief Get coordinates after a downward movement
         * @param none
         * @return Coordinate of the next node
         */
        uint16_t GoDown();

        /**
         * @brief Get coordinates after a movement towards right direction
         * @param none
         * @return Coordinate of the next node
         */
        uint16_t GoRight();

        /**
         * @brief Get coordinates after a movement towards left direction
         * @param none
         * @return Coordinate of the next node
         */
        uint16_t GoLeft();

        /**
         * @brief Get coordinates after an upward movement
         * @param none
         * @return Coordinate of the next node
         */
        uint16_t GoUpRight();

        /**
         * @brief Get coordinates after an upward movement
         * @param none
         * @return Coordinate of the next node
         */
        uint16_t GoUpLeft();

        /**
         * @brief Get coordinates after an upward movement
         * @param none
         * @return Coordinate of the next node
         */
        uint16_t GoDownRight();

        /**
         * @brief Get coordinates after an upward movement
         * @param none
         * @return Coordinate of the next node
         */
        uint16_t GoDownLeft();

    public:
        const uint8_t kMaxNumActions = 8;

        /**
         * @brief Constructor for the class
         * @param none
         * @return none
         */
        Actions();

        /**
         * @brief Destructor for the class
         * @param none
         * @return none
         */
        ~Actions();

        /**
         * @brief Get coordinates after an upward movement
         * @param prevCoord Coordinate of the prevoius node
         * @param action Direction in which the robot will move
         * @param type Defines whether it is an x or y co-ordinate
         * @return Coordinate of the next node
         */
        int GetNextCoord(uint16_t prevCoord, uint8_t action, char type = 'x');
};

#endif  //  INCLUDE_ACTIONS_ACTIONS_H_
