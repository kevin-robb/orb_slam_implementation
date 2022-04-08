# eece5554-vslam
Final group project for EECE-5554 Robotics Sensing &amp; Navigation, involving Visual SLAM using data from the NUANCE autonomous car.

Check out this link for a similar example:
[Monocular Visual Odometry using OpenCV](https://avisingh599.github.io/vision/monocular-vo/)

![dataset_preview](https://user-images.githubusercontent.com/83112082/161122619-1058b1d1-7834-442d-bf6a-d888db4a47ef.gif)

[How to export image and video data from a bag file](http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data)


# ORB-SLAM-3 Setup on Ubuntu 20.04

First ensure that Python is installed on your system, and that you have the `numpy` library installed. We will be using ROS Noetic in this project, so that should be installed as well.

## Install all library dependencies.

    sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
    sudo apt update

    sudo apt-get install build-essential
    sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

    sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev

    sudo apt-get install libglew-dev libboost-all-dev libssl-dev

    sudo apt install libeigen3-dev

## Install OpenCV

    cd ~
    mkdir Dev
    cd Dev

In the `Dev` directory, git clone the opencv repository from [this link](https://github.com/opencv/opencv). Using SSH, this looks like `git clone git@github.com:opencv/opencv.git`.

Next, open the header file with

    cd opencv
    nano ./modules/videoio/src/cap_ffmpeg_impl.hpp

In this file, paste the following lines at the top.

    #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
    #define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
    #define AVFMT_RAWPICTURE 0x0020

Save and close the file. Now, build it with the following set of commands.

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DWITH_CUDA=OFF -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_PRECOMPILED_HEADERS=OFF ..
    make -j 3
    sudo make install

## Install Pangolin

    cd ~/Dev

In the `Dev` directory, git clone the pangolin repository from [this link](https://github.com/stevenlovegrove/Pangolin). Using SSH, this looks like `git clone --recursive git@github.com:stevenlovegrove/Pangolin.git`.

    cd Pangolin
    mkdir build
    cd build
    cmake ..
    cmake --build .
    make -j 3
    sudo make install

## Install Eigen C++ Library

Download the latest release from [this link](http://eigen.tuxfamily.org/) as a zip file. As of writing this, this is version 3.4.0. Extract it, and move the resulting folder to `~/Dev`. Then run the following commands.

    cd ~/Dev/eigen-3.4.0/
    mkdir build
    cd build
    cmake ..
    sudo make install

## Install ORB-SLAM-3

The [official ORB_SLAM3 repo](https://github.com/UZ-SLAMLab/ORB_SLAM3) at the time of me writing this does not work with ROS, at least when following their README. There is a [forked repo](https://github.com/nindanaoto/ORB_SLAM3) that fixes the issues, so we will use it instead.

Navigate once again to `~/Dev`, and git clone the repo.

    cd ~/Dev
    git clone git@github.com:nindanaoto/ORB_SLAM3.git

To make ORB SLAM work with ROS, we need to edit the bashrc with `nano ~/.bashrc`, and add the following line at the end:

    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Dev/ORB_SLAM3/Examples/ROS

Make sure to re-source the bashrc to reflect its changes.

    source ~/.bashrc

Next, we will build ORB-SLAM for ROS.

    cd ~/Dev/ORB_SLAM3/Examples/ROS/ORB_SLAM3
    mkdir build
    cd build
    cmake .. -DROS_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=`which python3`
    make -j

---

I'm having errors at the very last `make -j` command. I've changed the C++ version from 11 to 14 in the CMakeLists in both the `ORB_SLAM3` and the `ORB_SLAM3/Examples/ROS/ORB_SLAM3` directories. I've also added the line `${PROJECT_SOURCE_DIR}/../../../Thirdparty` to the latter CMakeLists in the include_directories() section, since the error seems to be that the config.h cannot be found for g2o in ThirdParty. 
