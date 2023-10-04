import csv
import matplotlib.pyplot as plt
import matplotlib
import os
import numpy as np
import sys

#FUNCTION
def checkValidCsv(headers):
    if len(headers) != 8:
        return False
    if headers[0] != "created_at":
        return False
    if headers[1] != "field1":
        return False
    if headers[2] != "field2":
        return False
    if headers[3] != "field3":
        return False
    if headers[4] != "field4":
        return False
    if headers[5] != "field5":
        return False
    if headers[6] != "field6":
        return False
    if headers[7] != "field7":
        return False
    return True

# Check if the number of arguments is correct
if len(sys.argv) != 2:
    print("Please provide exactly one argument for the file path.")
    sys.exit(1)

# Get the file path from the terminal arguments
filePath = sys.argv[1]

# Check if filePath exists
if not os.path.exists(filePath):
    print("Please provide a valid file path.")
    sys.exit(1)

# Extract the file name from the file path
fileName = os.path.basename(filePath)

# Split file name and extension
fileNameSplitted = os.path.splitext(fileName)

# Check if the file is a CSV file
if fileNameSplitted[1] != ".csv":
    print("Please provide a CSV file.")
    sys.exit(1)

# Open the CSV file and read the data
with open(os.path.join("./logs",fileName), 'r') as file:
    reader = csv.reader(file)
    data = list(reader)

# Check if the CSV file is valid
if not checkValidCsv(data[0]):
    print("Please provide a valid CSV file.")
    sys.exit(1)

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
plt.savefig("./logs/" + fileNameSplitted[0] + ".png")