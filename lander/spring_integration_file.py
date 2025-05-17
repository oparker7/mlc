import numpy as np
import matplotlib.pyplot as plt


m=1
k=1
x=0
v=1

def integrate_euler_verlet(m, k, x, v, dt, method="euler"):
    t_max = 100
    t_array = np.arange(0, t_max, dt)
    num_steps = len(t_array)
    
    x_array = np.zeros(num_steps)
    v_array = np.zeros(num_steps)
    
    x_array[0] = x
    v_array[0] = v
    
    for i, t in enumerate(t_array[:-1]):
        a = -k * x_array[i] / m
        
        if method == "euler":
            x_array[i+1] = x_array[i] + dt * v_array[i]
            v_array[i+1] = v_array[i] + dt * a
        elif method == "verlet":
            x_array[i+1] = 2 * x_array[i] - x_array[i-1] + a * dt ** 2
            v_array[i+1] = (x_array[i+1] - x_array[i]) / dt
    
    return x_array, v_array

def assess_timeperiod(f, method_name):
    timeperiod = np.linspace(-5, 2, 20)
    final_displacements = []

    for dt in timeperiod:
        x_t, _ = f(m, k, x, v, dt)
        final_displacements.append(x_t[-1])

    plt.figure()
    plt.xlabel('dt (s)')
    plt.ylabel('final displacement')
    plt.grid()
    plt.plot(timeperiod, final_displacements, label=method_name)
    plt.legend()
    plt.show()

method_names = ["euler", "verlet"]
for method_name in method_names:
    assess_timeperiod(integrate_euler_verlet, method_name)