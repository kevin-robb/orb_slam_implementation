# eece5554-vslam
Final group project for EECE-5554 Robotics Sensing &amp; Navigation, involving Visual SLAM using data from the NUANCE autonomous car.
<!-- 
Check out this link for a similar example:
[Monocular Visual Odometry using OpenCV](https://avisingh599.github.io/vision/monocular-vo/) -->

![dataset_preview](https://user-images.githubusercontent.com/83112082/161122619-1058b1d1-7834-442d-bf6a-d888db4a47ef.gif)

# ORB-SLAM-3 Setup

You will have the best chance of ORB SLAM 3 working if you follow these directions exactly, including using a fresh Ubuntu 20.04 install on a newly created virtual machine.

Ensure your machine has at least 16GB of RAM.

Install the [VMware Workstation Player](https://www.vmware.com/hk/products/workstation-player.html) virtual machine. I did this on my Windows desktop PC. During setup, customize the hardware to use the max amount of recommended RAM, and 4 processor cores.

Download the [Ubuntu 20.04 iso](https://ubuntu.com/download/desktop).

After setting up the OS, you may want to connect a USB device, such as a drive you have data stored on. You can do this via the top-left tab, but by default all options are grayed out. To fix this, power down the VM, navigate to your VM (mine is in `Documents/Virtual Machines`), and open the .vmx file with a text editor. Delete the following line

    usb.restrictions.defaultAllow = “FALSE”

Now reopen the VM through the VMware launcher, and USB devices should be selectable.

## Install dependencies

    sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
    sudo apt update
    sudo apt-get install build-essential
    sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev
    sudo apt-get install libglew-dev libboost-all-dev libssl-dev
    sudo apt install libeigen3-dev
    sudo apt-get install libcanberra-gtk-module

## Install OpenCV
You will need two versions of OpenCV installed, 4.2.0 and 3.2.0. First install 4.2.0:

    cd ~
    mkdir Dev && cd Dev
    git clone https://github.com/opencv/opencv.git
    cd opencv
    git checkout 4.2.0

Run `nano ./modules/videoio/src/cap_ffmpeg_impl.hpp` or your favorite text editor and add the following three lines at the top of the file:

    #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
    #define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
    #define AVFMT_RAWPICTURE 0x0020

Now build it.

    mkdir build
    cd build
    cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
    make -j 3
    sudo make install
    cd ..
    mv opencv opencv4

Now install the older version:

    cd ~
    mkdir Dev && cd Dev
    git clone https://github.com/opencv/opencv.git
    cd opencv
    git checkout 3.2.0

Similarly, run `nano ./modules/videoio/src/cap_ffmpeg_impl.hpp` or your favorite text editor and add the following three lines at the top of the file:

    #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
    #define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
    #define AVFMT_RAWPICTURE 0x0020

Now build it.

    mkdir build
    cd build
    cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
    make -j 3
    sudo make install

## Install Pangolin
We will use an older commit version of Pangolin for compatibility.

    cd ~/Dev
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin 
    git checkout 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d
    mkdir build 
    cd build 
    cmake .. -D CMAKE_BUILD_TYPE=Release 
    make -j 3 
    sudo make install

## Install ORB_SLAM3
We will use an older version as well.

    cd ~/Dev
    git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git 
    cd ORB_SLAM3
    git checkout ef9784101fbd28506b52f233315541ef8ba7af57

This version has some compilation errors that we need to fix.
Run `nano ./include/LoopClosing.h` or your favorite text editor and change line 51 from 

    Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

to

    Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;

In `CMakeLists.txt`, change the OpenCV version line to

    find_package(OpenCV 3.2)

We also need to make changes to `System.cc`. Open it with `nano ./src/System.cc` or your favorite text editor, and change both lines 584 and 701 from

    Map* pBiggerMap;

to

    Map* pBiggerMap = nullptr;

This latter error allowed the examples to run fine, but they caused a segmentation fault at the very end when trying to save the map.

Now all compilation errors should be fixed, and we can build ORB_SLAM3. Run their provided shell script `build.sh`, which will build all of the third party programs as well as ORB_SLAM3 itself. 

    ./build.sh

It will likely show errors, but just run it a few times without changing anything, and it should succeed after a few attempts.

## Download Example Data
We will use the EuRoC MH_01 easy dataset.

    cd ~
    mkdir -p Datasets/EuRoc
    cd Datasets/EuRoc/
    wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
    mkdir MH01
    unzip MH_01_easy.zip -d MH01/

Two of the images in my download were corrupted, so for the program to run successfully, I replaced each of these with their nearest frame.

    cd ~/Datasets/EuRoc/MH01/mav0/cam0/data
    rm 1403636689613555456.png
    cp 1403636689663555584.png 1403636689613555456.png
    rm 1403636722213555456.png
    cp 1403636722263555584.png 1403636722213555456.png

If the example exits with a segmentation fault, retry the unzip step for your data, and pay attention for an image failing with a bad CRC; the image is likely corrupted and should be replaced in the manner I've done here. Note that each image must appear by name, so corrupt images must be replaced by an adjacent frame rather than simply deleted.

## Run Simulation with Examples

    cd ~/Dev/ORB_SLAM3

Then, choose one of the following to run. A map viewer as well as an image viewer should appear after it finishes setup.

    # Mono
    ./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

    # Mono + Inertial
    ./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

    # Stereo
    ./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

    # Stereo + Inertial
    ./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi

## Validating Estimates vs Ground Truth
We're using python 2.7, and need numpy and matplotlib. For this, we need the 2.7 version of pip.

    sudo apt install curl
    cd ~/Desktop
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
    sudo python2 get-pip.py
    pip2.7 install numpy matplotlib

Now run and plot the ground truth:

    cd ~/Dev/ORB_SLAM3

    ./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

And now run and plot the estimate compared to ground truth:

    cd ~/Dev/ORB_SLAM3

    python evaluation/evaluate_ate_scale.py evaluation/Ground_truth/EuRoC_left_cam/MH01_GT.txt f_dataset-MH01_stereo.txt --plot MH01_stereo.pdf

Open the pdf `MH01_stereo.pdf` to see the results. This can be done with the command `evince MH01_stereo.pdf`.


# Using Our Own Data
Since we have not setup the ROS-compatible part of ORB_SLAM3, the only input we can use is a folder of images, with a corresponding txt file of timestamps. To convert a rosbag of image data into this format, follow [this guide](http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data).

