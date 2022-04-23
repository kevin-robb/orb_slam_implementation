#!/bin/bash

nameUser='USERNAME'
nameDrive='DRIVE'
nameDataset='DATASET_NAME'

echo "Template Stereo"

pathYaml='~/../../media/'$nameUser'/'$nameDrive'/DATA/'$nameDataset'/nuance_car.yaml'
pathDataset='~/../../media/'$nameUser'/'$nameDrive'/DATA/'$nameDataset'/images'
pathTimestamps='~/../../media/'$nameUser'/'$nameDrive'/DATA/'$nameDataset'/timestamps.txt'
outputName='your_desired_output_dataset_name'

./~/Dev/ORB_SLAM3/Examples/Stereo/stereo_euroc ~/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt "$pathYaml" "$pathDataset" "$pathTimestamps" "$outputName"