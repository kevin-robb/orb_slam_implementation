#!/bin/bash

echo "Launching Set 1 Stereo"

pathYaml='~/../../media/kevinrobb/ROBB0005/DATA/nuance_car/nuance_car.yaml'
pathDataset='~/../../media/kevinrobb/ROBB0005/DATA/nuance_car/car_provided/images'
pathTimestamps='~/../../media/kevinrobb/ROBB0005/DATA/nuance_car/car_provided/timestamps.txt'
outputName='dataset_car_provided_stereo'

./~/Dev/ORB_SLAM3/Examples/Stereo/stereo_euroc ~/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt "$pathYaml" "$pathDataset" "$pathTimestamps" "$outputName"
