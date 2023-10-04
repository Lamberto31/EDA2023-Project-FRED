#!/bin/bash

# Loop through all csv files with "FRED_log" in the name
for file in FRED_log*.csv; do
    # Count the number of lines in the file
    lines=$(wc -l < "$file")
    # If there is only one line, delete the file
    if [ "$lines" -eq 1 ]; then
        rm "$file"
    fi
done
