#!/bin/bash

echo "Run this from ~ or it won't work! (cd ~)"

# Customize these for your dataset
# nameUser='USERNAME'
# nameDrive='DRIVE'
# nameDataset='DATASET_NAME'
# nameYaml='YOUR_YAML.yaml'
# outputName='dataset_TEMPLATE'

# Set 2 Monocular params:
nameUser='kevinrobb'
nameDrive='ROBB0005/DATA'
nameDataset='nuance_car/car_new'
nameYaml='nuance_car.yaml'
outputName='dataset_car_new_mono'


pathYaml='./../../media/'$nameUser'/'$nameDrive'/'$nameDataset'/'$nameYaml
pathDataset='./../../media/'$nameUser'/'$nameDrive'/'$nameDataset'/images'
pathTimestamps='./../../media/'$nameUser'/'$nameDrive'/'$nameDataset'/timestamps.txt'
pathVocab='./Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt'


# echo "Template Stereo"
# ./~/Dev/ORB_SLAM3/Examples/Stereo/stereo_euroc "$pathVocab" "$pathYaml" "$pathDataset" "$pathTimestamps" "$outputName"

echo "Template Monocular"
./Dev/ORB_SLAM3/Examples/Monocular/mono_euroc "$pathVocab" "$pathYaml" "$pathDataset" "$pathTimestamps" "$outputName"
