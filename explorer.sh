#!/bin/sh
# Create build directory if it doesn't exists
if [ ! -d "$PWD/build/" ]; then
    mkdir build/
fi
# Shift to the build directory
cd build/
# Remove prevoius versions of the text files
if [ -d "path/" ]; then
    rm -rf path/
fi
# Create directory to store text files
mkdir path/
# Configure and compile the project
cmake ..
cmake --build .