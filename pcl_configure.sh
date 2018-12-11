#!/bin/bash

git clone https://github.com/cirpote/pcl
cd pcl
git checkout pcl-1.8.0
mkdir build && cd build
cmake -DBUILD_registration=false -DCMAKE_BUILD_TYPE=Release -DBUILD_surface=false -DBUILD_rage_image=false ..
make -j6
