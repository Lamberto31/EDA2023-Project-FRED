#!/bin/bash

# Change to the directory where the csv files are located
cd ./logs

# Counter for the number of empty files
count=0

# Loop through all csv files with "FRED_" in the name
for file in FRED_*.csv; do
    # Count the number of lines in the file
    lines=$(wc -l < "$file")
    # If there is only one line, delete the file
    if [ "$lines" -eq 1 ]; then
        count=$((count+1))
        rm "$file"
    fi
done

# Print the number of files deleted
if [ "$count" -eq 1 ]; then
    echo "$count log file deleted"
else
    echo "$count log files deleted"
fi