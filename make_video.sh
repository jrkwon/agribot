#!/bin/bash

if [ $# -eq 0 ]; then 
  echo "Usage: $0 <path/to/images>"
  exit 1
fi

input_directory=$1
input_directory="${input_directory%/}"

output_file="${input_directory}.mp4"

ffmpeg -framerate 30 -pattern_type glob -i "${input_directory}/*.jpg" -c:v libx264 -pix_fmt yuv420p "${output_file}"

echo "Video created: ${output_file}"
