#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {

   // declare variables
   double m, k, x, v, t_max, dt, t, a;
   vector<double> t_list, x_list, v_list;

   // mass, spring constant, initial position and velocity
   m = 1;
   k = 1;
   x = 0;
   v = 1;

   // simulation time and timestep
   t_max = 1000;
   dt = 0.0001;

   // Initialize previous position
   a = -k * x / m;
   double prev_x = x - v * dt + 0.5 * a * dt * dt;

   // Verlet integration
   for (t = 0; t <= t_max; t = t + dt) {

       // append current state to trajectories
       t_list.push_back(t);
       x_list.push_back(x);
       v_list.push_back(v);

       // calculate new position and velocity
       a = -k * x / m;
       double new_x = 2 * x - prev_x + a * dt * dt;
       v = (new_x - prev_x) / (2 * dt);

       // Update previous position
       prev_x = x;
       x = new_x;

    }

  // Write the trajectories to file
  ofstream fout;
  fout.open("verlet_trajectories.txt");
  if (fout) { // file opened successfully
    for (int i = 0; i < t_list.size(); i = i + 1) {
      fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
    }
  } else { // file did not open successfully
    cout << "Could not open trajectory file for writing" << endl;
  }
 
  /* The file can be loaded and visualised in Python as follows:

  import numpy as np
  import matplotlib.pyplot as plt
  results = np.loadtxt('trajectories.txt')
  plt.figure(1)
  plt.clf()
  plt.xlabel('time (s)')
  plt.grid()
  plt.plot(results[:, 0], results[:, 1], label='x (m)')
  plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
  plt.legend()
  plt.show()

  */
}
