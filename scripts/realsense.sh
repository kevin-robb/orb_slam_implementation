#!/bin/bash
# Run ORB_SLAM3 with settings for the RealSense camera on a specified dataset.

echo "Usage: ./realsense.sh DATASET_NAME"

echo "Run this from ~ or it won't work! (cd ~)"
echo "---"

# Customize these for your dataset
# nameUser='USERNAME'
# nameDrive='DRIVE'
# nameDataset='DATASET_NAME'
# nameYaml='YOUR_YAML.yaml'
# outputName='dataset_TEMPLATE'

# -----------------------------------------
# Specific params for our VM:
nameUser='kevinrobb'
nameDrive='ROBB0005/DATA/realsense'
nameYaml='realsense.yaml'

# Dataset:
nameDataset=$1

# Output Filename:
nameOutput=$1

# ------------------------------------------
# Set filepaths based on specified params.
pathYaml='./../../media/'$nameUser'/'$nameDrive'/'$nameYaml
# pathYaml='./../../media/'$nameUser'/'$nameDrive'/'$nameDataset'/'$nameYaml
pathDataset='./../../media/'$nameUser'/'$nameDrive'/'$nameDataset'/images'
pathTimestamps='./../../media/'$nameUser'/'$nameDrive'/'$nameDataset'/timestamps.txt'
pathVocab='./Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt'

# ------------------------------------------
# Run the thing.

echo "Running in Monocular"
./Dev/ORB_SLAM3/Examples/Monocular/mono_euroc "$pathVocab" "$pathYaml" "$pathDataset" "$pathTimestamps" "$nameOutput"
