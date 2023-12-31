#!/bin/bash

# Check if an argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <directory_path>"
    exit 1
fi

directory_path="$1"
directory_path="${directory_path%/}"

# Use basename to extract the last filename
last_filename=$(basename "${directory_path}")

echo "Created a symbolic link for ${directory_path}.csv"
ln -s $last_filename/$last_filename.csv $last_filename.csv
mv $last_filename.csv $1/../.