#!/bin/bash

if [ "$MBZIRC_DIR" == "" ]
then
  echo "You need to set the MBZIRC_DIR variable for this script."
  exit
fi

echo 'Environment variables configured. Installing OpenCV 3.2.0 ...'
sleep 1

cd "$MBZIRC_DIR"
mkdir -p "tmp"

# Clone OpenCV repos
cd "$MBZIRC_DIR/tmp"
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

# Checkout 3.2.0 tag
cd "$MBZIRC_DIR/tmp/opencv"
git checkout tags/3.2.0
cd "$MBZIRC_DIR/tmp/opencv_contrib"
git checkout tags/3.2.0

# Configure OpenCV installation
cd "$MBZIRC_DIR/tmp/opencv"
mkdir -p "build"
cd "$MBZIRC_DIR/tmp/opencv/build"
cmake \
  -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D WITH_CUDA=ON \
  -D ENABLE_FAST_MATH=1 \
  -D CUDA_FAST_MATH=1 \
  -D WITH_CUBLAS=1 \
  -D INSTALL_PYTHON_EXAMPLES=OFF \
  -D OPENCV_EXTRA_MODULES_PATH="$MBZIRC_DIR/tmp/opencv_contrib/modules" \
  -D CUDA_CUDA_LIBRARY=/usr/local/cuda/lib64/stubs/libcuda.so \
  -D BUILD_EXAMPLES=ON ..

# Install OpenCV
make
sudo make install
