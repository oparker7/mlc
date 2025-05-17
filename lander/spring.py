# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
import time

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

def euler_spring(m, k, x, v, dt=0.0001):

    # Start time
    start_time = time.time()


    # simulation time, timestep and time
    t_max = 1000
    t_array = np.arange(0, t_max, dt)
    # initialise empty lists to record trajectories
    x_list = []
    v_list = []
    # Euler integration
    for t in t_array:
        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)
        # calculate new position and velocity
        a = -k * x / m
        x = x + dt * v
        v = v + dt * a
    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    # End time
    end_time = time.time()

    # Calculate elapsed time
    elapsed_time = end_time - start_time
    print("Elapsed time:", elapsed_time, "seconds")

    # plot the position-time graph
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(t_array, x_array, label='x (m)')
    plt.plot(t_array, v_array, label='v (m/s)')
    plt.legend(loc = 'upper left')
    plt.show()

    return (x)


def verlet_spring(m, k, x, v, dt=0.1):
    # initialise empty lists to record trajectories
    x_list = [0, 0.1]
    v_list = [1, 0.99]

    # simulation time, timestep and time
    t_max = 100
    t_array = np.arange(0, t_max, dt)

    # Verlet integration
    for t in range(len(t_array)-2):

        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        #calculate new position and velocity
    
        a = - k * x_list[-1] / m
        x = 2 * x_list[-1] - x_list[-2] + a * dt ** 2
        v = (x - x_list[-1]) / dt

    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    # plot the position-time graph
    plt.figure(2)
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(t_array, x_array, label='x (m)')
    plt.plot(t_array, v_array, label='v (m/s)')
    plt.legend()
    plt.show()


def assess_timeperiod(f):
    timeperiod = np.linspace(-5, 2, 20, 2)

    #initialise list of final displacements
    final_displacements = []

    for dt in timeperiod:
        x_t = f(m, k, x, v, dt)
        final_displacements.append(x_t)

    plt.clf()
    plt.figure(3)
    plt.xlabel('dt (s)')
    plt.ylabel('final displacement')
    plt.grid()
    plt.plot(timeperiod, final_displacements, label='x (m)')
    plt.show()



euler_spring(m,k,x,v)
