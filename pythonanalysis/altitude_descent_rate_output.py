import matplotlib.pyplot as plt
import numpy as np
# Lists to store the data
h_list = []
ver_list = []


# Read data from the file
with open("C:\\Users\\Olly\\source\\repos\\lander\\lander\\heights.txt", "r") as fin:
    for line in fin:
        # Split the line into two values
        h, ver = map(float, line.split())
        h_list.append(h)
        ver_list.append(ver)

# Create the graph
plt.figure(figsize=(8, 6))
plt.plot(h_list, ver_list, marker='o', linestyle='-')
plt.title("Altitude vs. Descent Rate")
plt.xlabel("Altitude")
plt.ylabel("Descent Rate")
plt.grid(True)


plt.show()