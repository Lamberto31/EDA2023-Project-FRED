import csv
import matplotlib.pyplot as plt
import matplotlib
import os
import numpy as np

# Open the CSV file and read the data
# TODO: rendere dinamico il nome del file tramite terminale
with open(os.path.join("./logs","FRED_log_2023-10-04T18:07:17.csv"), 'r') as file:
    reader = csv.reader(file)
    data = list(reader)

# Extract the x and ys data from the CSV file
timestamps = np.array([row[0] for row in data][1:], int)
fields = np.array([row[1:] for row in data][1:], float)

# Normalize the timestamps
timestamps = np.interp(timestamps, (timestamps[0], timestamps[-1]), (1, len(timestamps)))

matplotlib.use('Agg')

# Plot the time series
for i in range(len(fields[0])):
    y = [row[i] for row in fields]
    plt.subplot(3, 3, i+1)
    plt.plot(timestamps, y, label=f"Field{i+1}", marker = 'o', markersize = 2)
    plt.xlabel('Time')
    plt.ylabel('Value')
    # plt.xticks([])
    plt.legend()

# Show the plot
plt.show()
plt.savefig("./logs/plot.png")