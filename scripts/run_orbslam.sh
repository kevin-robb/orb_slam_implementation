#!/bin/bash

echo "Run this from ~ or it won't work! (cd ~)"
echo "Uncomment sections in this script for what you want to run."

# Customize these for your dataset
# nameUser='USERNAME'
# nameDrive='DRIVE'
# nameDataset='DATASET_NAME'
# nameYaml='YOUR_YAML.yaml'
# outputName='dataset_TEMPLATE'

# -----------------------------------------
# Specific params for our VM:
nameUser='kevinrobb'
nameDrive='ROBB0005/DATA'
nameYaml='nuance_car.yaml'

# Dataset:
# nameDataset='nuance_car/car_provided'
nameDataset='nuance_car/car_new'

# Output Filename:
# outputName='dataset_car_provided_stereo'
# outputName='dataset_car_provided_mono'
# outputName='dataset_car_new_stereo'
outputName='dataset_car_new_mono'

# ------------------------------------------
# Set filepaths based on specified params.
pathYaml='./../../media/'$nameUser'/'$nameDrive'/'$nameDataset'/'$nameYaml
pathDataset='./../../media/'$nameUser'/'$nameDrive'/'$nameDataset'/images'
pathTimestamps='./../../media/'$nameUser'/'$nameDrive'/'$nameDataset'/timestamps.txt'
pathVocab='./Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt'

# ------------------------------------------
# Run the thing.

# echo "Running in Stereo"
# ./~/Dev/ORB_SLAM3/Examples/Stereo/stereo_euroc "$pathVocab" "$pathYaml" "$pathDataset" "$pathTimestamps" "$outputName"

echo "Running in Monocular"
./Dev/ORB_SLAM3/Examples/Monocular/mono_euroc "$pathVocab" "$pathYaml" "$pathDataset" "$pathTimestamps" "$outputName"
