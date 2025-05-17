import numpy as np
import matplotlib.pyplot as plt
m = 1
M = 642E21
G = 667E-13
# Euler Method
def euler_orbital_trajectory(v_x):
    # mass, spring constant, initial position and velocity

    position = np.array([0, 0, 35786000])
    velocity = np.array([v_x, 0, 0])

    # simulation time, timestep and time
    t_max = 100000
    dt = 0.1
    t_array = np.arange(0, t_max, dt)

    # initialise empty lists to record trajectories
    position_list = []
    velocity_list = []

    # Euler integration
    for t in t_array:

        # append current state to trajectories
        position_list.append(position)
        velocity_list.append(velocity)

        # calculate new position and velocity
        r = np.linalg.norm(position)
        a = - G * M * position / r**3
        velocity = velocity + dt * a
        position = position + dt * velocity


        if r < 6371000:
            break
    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    pos_array = np.array(position_list)


    x_pos = [element[0] for element in pos_array]
    z_pos = [element[2] for element in pos_array]


    # Circle parameters
    h = 0      # x-coordinate of center
    k = 0      # y-coordinate of center
    r = 6371000  # radius

    # Generate angles
    theta = np.linspace(0, 2*np.pi, 100)

    # Parametric equations for the circle
    x = h + r * np.cos(theta)
    y = k + r * np.sin(theta)

    
    # plot the position-time graph
    plt.figure(1)
    plt.clf()
    plt.axis('equal')  # Set aspect ratio to be equal
    plt.xlabel('x position (m)')
    plt.ylabel('z position (m)')
    plt.grid()
    plt.plot(x_pos, z_pos, label='position (m)')
    plt.plot(x, y, label = "Earth's Surface")
    plt.legend()
    plt.show()

    


euler_orbital_trajectory(1000)