#!/bin/sh

mkdir build
cd build
cmake -D WITH_TBB=ON -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_OPENCL=OFF -D WITH_CUDA=OFF -D BUILD_opencv_gpu=OFF -D BUILD_opencv_nonfree=OFF -D BUILD_opencv_stitching=OFF -D BUILD_opencv_superres=OFF ..

make -j8

sudo make install