import numpy as np
import matplotlib.pyplot as plt
import time

# Start time
start_time = time.time()

file_path = r"C:\Users\Olly\source\repos\Spring\Spring\trajectories.txt"
results = np.loadtxt(file_path)


# End time
end_time = time.time()
# Calculate elapsed time
elapsed_time = end_time - start_time

print("Elapsed time:", elapsed_time, "seconds")
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='x (m)')
plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
plt.legend(loc = 'upper left')
plt.show()


# file path for trajectories.txt is "C:\Users\Olly\source\repos\Spring\Spring\trajectories.txt"