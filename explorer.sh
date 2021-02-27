#!/bin/sh
# Create build directory if it doesn't exists
if [ ! -d "$PWD/build/" ]; then
    mkdir build/
fi
# Shift to the build directory
cd build/
# Configure and compile the project
cmake ..
cmake --build .