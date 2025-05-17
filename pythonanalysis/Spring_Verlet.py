
import matplotlib.pyplot as plt


def verlet_integration(k, m, x0, v0, dt, num_steps):
    # Initialize lists to store positions and velocities
    positions = [x0]
    velocities = [v0]

    # Perform the Verlet integration for num_steps
    for _ in range(num_steps):
        # Current position and velocity
        x_curr = positions[-1]
        v_curr = velocities[-1]

        # Use the Verlet method to calculate the next position and velocity
        x_next = x_curr + v_curr * dt + 0.5 * (-k * x_curr / m) * dt**2
        v_next = v_curr + 0.5 * ((-k * x_curr / m) + (-k * x_next / m)) * dt

        # Append the next position and velocity to the lists
        positions.append(x_next)
        velocities.append(v_next)

    return positions, velocities

# Example usage
k = 1.0  # spring constant
m = 1.0  # mass
x0 = 0.0  # initial position
v0 = 1.0  # initial velocity
dt = 0.01  # time step
num_steps = 1000  # number of steps

positions, velocities = verlet_integration(k, m, x0, v0, dt, num_steps)

# Create a list of time values
time_values = [i * dt for i in range(num_steps + 1)]

# Plotting the results
plt.figure(figsize=(12, 6))

# Plotting the position vs time
plt.subplot(1, 2, 1)
plt.plot(time_values, positions)
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Position vs Time')

# Plotting the velocity vs time
plt.subplot(1, 2, 2)
plt.plot(time_values, velocities)
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Velocity vs Time')

plt.tight_layout()
plt.show()