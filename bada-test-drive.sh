#!/bin/bash

## no-crop
echo "Change neural_net: section to have scout-bada-orchard-no-crop in config/config-<id>.yaml"
echo "Press any key to continue when ready..."
read -n 1 -s
python neural_net/test_drive.py e2e-dataset/bada-all/2024-01-04-16-47-00_scout-bada-orchard-no-crop_n3_2024-01-04-18-06-58-372.keras e2e-dataset/bada-all/2024-01-04-16-47-00

## bottom-half-crop
echo "Change neural_net: section to have scout-bada-orchard-bottom-half-crop in config/config-<id>.yaml"
echo "Press any key to continue when ready..."
read -n 1 -s
python neural_net/test_drive.py e2e-dataset/bada-all/2024-01-04-16-47-00_scout-bada-orchard-bottom-half-crop_n3_2024-01-05-14-02-14-884.keras e2e-dataset/bada-all/2024-01-04-16-47-00

## bottom-half-upper-half-crop
echo "Change neural_net: section to have scout-bada-orchard-bottom-half-upper-half-crop in config/config-<id>.yaml"
echo "Press any key to continue when ready..."
read -n 1 -s
python neural_net/test_drive.py e2e-dataset/bada-all/2024-01-04-16-47-00_scout-bada-orchard-bottom-half-upper-half-crop_n3_2024-01-05-16-01-04-794.keras e2e-dataset/bada-all/2024-01-04-16-47-00

## area-crop
echo "Change neural_net: section to have scout-bada-orchard-area-crop in config/config-<id>.yaml"
echo "Press any key to continue when ready..."
read -n 1 -s
python neural_net/test_drive.py e2e-dataset/bada-all/2024-01-04-16-47-00_scout-bada-orchard-area-crop_n3_2024-01-06-15-53-11-230.keras e2e-dataset/bada-all/2024-01-04-16-47-00

