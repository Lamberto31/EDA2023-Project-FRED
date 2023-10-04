import csv
import matplotlib.pyplot as plt
import matplotlib
import os
import numpy as np
import sys

# Check if the number of arguments is correct
if len(sys.argv) != 2:
    print("Please provide exactly one argument for the file path.")
    sys.exit(1)

# Get the file path from the terminal arguments
filePath = sys.argv[1]

# Extract the file name from the file path
fileName = os.path.basename(filePath)

# Extract file name withtout extension
fileNameNoExt = os.path.splitext(fileName)[0]

# Open the CSV file and read the data
with open(os.path.join("./logs",fileName), 'r') as file:
    reader = csv.reader(file)
    data = list(reader)

# Extract the x and ys data from the CSV file
timestamps = np.array([row[0] for row in data][1:], int)
fields = np.array([row[1:] for row in data][1:], float)

# Normalize the timestamps
x = np.interp(timestamps, (timestamps[0], timestamps[-1]), (1, len(timestamps)))

matplotlib.use('Agg')

# Plot the time series
for i in range(len(fields[0])):
    y = [row[i] for row in fields]
    plt.subplot(3, 3, i+1)
    plt.plot(x, y, label=f"Field{i+1}", marker = 'o', markersize = 2)
    plt.xlabel('Time')
    plt.ylabel('Value')
    # plt.xticks([])
    plt.legend()

# Show the plot
plt.show()
plt.savefig("./logs/" + fileNameNoExt + ".png")