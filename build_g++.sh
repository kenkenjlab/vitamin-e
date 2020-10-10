#!/bin/sh

mkdir build_g++
cd build_g++
cmake -DCMAKE_CXX_COMPILER=g++-8 ../src
make -j3
