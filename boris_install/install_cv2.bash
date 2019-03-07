#!/bin/bash

# Assumption: this script assumes we are running it in fresh install of Ubuntu 14.04
# (Ubuntu 16.04 should work too)

sudo apt-get update

# install opencv-2.4.13.4
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    python-dev \
    python-numpy \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libjasper-dev \
    libdc1394-22-dev && \
    wget https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.13/opencv-2.4.13.zip && \
    unzip opencv-2.4.13.zip && \
    cd opencv-2.4.13/ && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_opencv_python=ON -D WITH_XINE=ON -D WITH_TBB=ON -DINSTALL_PYTHON_EXAMPLES=ON -DBUILD_EXAMPLES=ON .. && \
    make -j$(nproc) && make install -j$(nproc) && \
    cd ../.. && \
    rm opencv-2.4.13.zip

sudo apt-get -y install python-opencv


# sudo pip install --upgrade pip
# sudo pip install numpy
# sudo pip install pybullet

# install opencv 2.4.13: https://gist.github.com/arthurbeggs/06df46af94af7f261513934e56103b30
