# Checklist for Collecting Data with the NUance Car.

## Before
Before even getting to the car, ensure you have:
 - External hard drive with at least 100 GB of storage.
 - One person to drive, and one person to sit in the backseat to control the data collection.
 - Ideally a third person to sit up front and serve as navigator.

When arriving at the car,
 - Start up the car and ensure all systems are turned on.
 - Login to auv with provided password.
 - Ensure the VLP LiDARs are turned on with the small screen by the front PRNDL.
 - Start the systems publishing data
    - `roslaunch agv agv_system_ouster.launch`
    - `roslaunch spinnaker_sdk_camera_driver acquisition.launch`
    - `cd /data_hdd/rsn`
    - Create directory with a team member's name, e.g. `mkdir TEAMNAME`. Record rosbags here.
    - Check that the `tf` and `tf_static` topics are being published to as well.
 - Check all the physical sensors you need:
    - Make sure camera lenses are cleaned.
    - Make sure LiDARs are spinning.
    - Get GPS puck out of glove box and connect to a laptop to get GPS data.

When starting data collection,
 - Make sure driver/navigator have a plan of the desired route.
 - Start rosbag recording. Ensure you are only recording things you need; the LiDAR data is very large and is not needed for vision-only projects such as ours.

## During
Person sitting with monitor should:
 - Keep checking `rostopic echo /TOPIC_OF_INTEREST` to ensure messages do not stop coming in.

Navigator should keep track of the route and be able to give directions to the driver, and replan in case of difficulties.

## After
After stopping the car, 
 - Stop the rosbag recording.
 - Open the trunk and plug in your external hard drive.
 - Copy the rosbag from `/data_hdd/rsn/TEAMNAME` to your drive. This takes a while since the data is so large.
 - If there is another group collecting data after you, don't turn off the car or stop the ROS stuff. If you're the last team, stop everything executing with the computer in the backseat, turn off the LiDARs, and turn off the car. 

As soon as you can, replay the rosbag you got on your machine to make sure the data looks good. Use `rviz` while it replays to select certain sensors like the cameras and LiDARs to see the live feed.


