This was a final project for EECE-5554: Robotics Sensing &amp; Navigation at Northeastern University, in which we implemented and used ORB_SLAM3 to perform Visual SLAM on a variety of data we collected ourselves, including data recorded with the NUance autonomous car. **However, this guide is completely general and can be used by anyone to collect data and process it using ORB_SLAM3.**

![dataset_preview](https://user-images.githubusercontent.com/83112082/161122619-1058b1d1-7834-442d-bf6a-d888db4a47ef.gif)

Our NUance car dataset is stored [here](https://northeastern-my.sharepoint.com/:u:/g/personal/robb_k_northeastern_edu/EfVkYY2cSmlIiCkSuT247QMBETdgZRNImgGfML5tXcE2yA?e=ZqOMIn). You must be logged in with a Northeastern University account to view and download it. 

# ORB-SLAM-3 Setup

You will have the best chance of ORB SLAM 3 working if you follow these directions exactly, including using a fresh Ubuntu 20.04 install on a newly created virtual machine.

Ensure your machine has at least 16GB of RAM and 4 cores. 32GB of RAM is recommended.

Install the [VMware Workstation Player](https://www.vmware.com/hk/products/workstation-player.html) virtual machine. I did this on my Windows 10 desktop PC. During setup, customize the hardware to use the max amount of recommended RAM, and 4 processor cores. Whenever you're running the VM, kill everything else running on the PC to allow as much RAM to be used by ORB_SLAM3 as possible.

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

---

# Using Our Own Data
I've done all of this section on a separate computer running Ubuntu 20.04 with ROS Noetic, and then copied the files onto an external storage drive, which I use with my virtual machine to run ORB_SLAM3. I would recommend doing this as well, rather than installing ROS onto the VM and potentially breaking things with version conflicts.

## Converting from Rosbag to Image Set
Since we have not setup the ROS-compatible part of ORB_SLAM3, the only input we can use is a folder of images, with a corresponding .txt file of timestamps. To convert a rosbag of image data into this format, use the `convert_images.py` script I've created. It will work for both monocular and stereo cameras. It's setup with the topics for a RealSense D435 monocular camera and stereo images from the NUance autonomous car, but additional modes can be easily added to the `cam_types` dictionary in `main()` of this script. Run it by executing each of the following commands in a separate terminal:

    roscore
    python3 scripts/convert_images.py DATASET_NAME CAM_TYPE
    rosbag play ROSBAG_NAME.bag

Here `DATASET_NAME` will be the name of the resulting folder, which will be created automatically in your `~` directory.

## Config and Stereo Rectification Parameters
We also need to provide ORB_SLAM3 a .yaml file containing calibration parameters for the cameras. Much of this information for the NUance data, including the D, K, R, and P matrices, is published on the `/camera_array/cam0/camera_info` and `/camera_array/cam1/camera_info` topics, so we can get it from echoing these topics and recording an output. The .yaml file I created for the car data is `config/nuance_car.yaml`. I've also created a .yaml for the RealSense camera at `config/realsense.yaml`.

## Running ORB_SLAM3 with Our Data
We now have everything we need to run ORB_SLAM3 with this dataset. If you performed this section on a different machine, now copy the relevant files onto an external drive to use in the virtual machine.

Let's assume your external drive is called `DRIVE`, and it has a folder in its main directory called `DATA`. The directory structure should then look like:

    DRIVE/
      DATA/
        DATASET_NAME/
          CONFIG.yaml
          timestamps.txt
            images/mav0/
              cam0/data/
                * all the left images
              cam1/data/
                * all the right images

So on the virtual machine, with the drive connected, you're almost ready to run ORB_SLAM3. Copy the bash script in `scripts/run_orbslam.sh` from this repository onto your virtual machine, and ensure it is executable with the command `chmod +x run_orbslam.sh`. Open it with your favorite text editor, and comment/uncomment the specified sections to set all the parameters accordingly. Then execute it:

    cd ~
    ./run_orbslam.sh


Alternatively, if you do not wish to create the bash file and copy its contents, you can run the following commands:

    cd ~/Dev/ORB_SLAM3

    ./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ~/../../media/USERNAME/DRIVE/DATA/DATASET_NAME/CONFIG.yaml ~/../../media/USERNAME/DRIVE/DATA/DATASET_NAME/images ~/../../media/USERNAME/DRIVE/DATA/DATASET_NAME/timestamps.txt your_desired_output_dataset_name

where you should fill in `USERNAME`, `DRIVE`, `DATASET_NAME`, and `CONFIG` for your case. To run monocular instead of stereo, replace the first argument with `./Examples/Monocular/mono_euroc`. The final command line argument will be used as part of the filename for the frame and keyframe trajectory files produced as output by ORB_SLAM3.

---

# Demos
I've used the contents of this repository to run ORB_SLAM3 on a variety of datasets I collected, as well as some of the demos. Videos of these are available on my [YouTube channel](https://www.youtube.com/channel/UCdxoB6xJBsmmi0iUU0E_yRA).

Some of these include:
 - [EuRoC example dataset](https://www.youtube.com/watch?v=mtbg8G4CSyw&t=180s) - Success
 - [NUance car dataset](https://www.youtube.com/watch?v=9lwM9MTMCQo&t=596s) - Failure
 - [RealSense small room](https://www.youtube.com/watch?v=VOHloE1mnos) - Success
 - [RealSense outdoor repeating environment](https://www.youtube.com/watch?v=EX5d3sjTI0w&list=PLQp5kRKQrSVYwbofYIjS4bbODDIJR0RMU&index=1) - Success
