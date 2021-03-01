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
 * @file errorLogger.h
 * @author Umang Rastogi
 * @brief Defines the ErrorLogger's class to return coordinates of the next node
 */

#ifndef INCLUDE_LOGGER_LOGGER_H_
#define INCLUDE_LOGGER_LOGGER_H_

#include <cstdint>
#include <string>

class ErrorLogger {
    private:
        uint8_t log_level;
        const std::string log_levels[4] = {"INFO", "WARN", "DEBUG", "FATAL"};
        
    public:
         /**
         * @brief Constructor for the class
         * @param log_level Sets the level of logging for the project
         * @return none
         */
        ErrorLogger(uint8_t log_lvl);

         /**
         * @brief Destructor for the class
         * @param none
         * @return none
         */
        ~ErrorLogger();

         /**
         * @brief Prints out error messages 
         * @param msg Message to be logged
         * @param log_lvl Level of error to be logged
         * @return none
         */
        void Log(std::string msg, uint8_t log_lvl);
};

#endif  // INCLUDE_LOGGER_LOGGER_H_