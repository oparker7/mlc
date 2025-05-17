// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include "math.h"
#include <vector>
vector3d g_force;
vector<double> h_list, ver_list;

void autopilot(void) {
// Autopilot to adjust the engine throttle, parachute and attitude control

    // INSERT YOUR CODE HERE

    // Proportional control governing equation is v . e_r = -(0.5 + K_h * h)
    

    double K_h = 0.017;
    double K_p = 0.7;
    double K_d = 1;

    // scenario 1 and scenario 4 land safely with kh as 0.02 and kp as 0.7
    // scenario 1 and scenario 5 land safely with kh as 0.017 and kp as 0.7

    double e_term;
    double altitude;
    double descent_rate;
    double P_out; // this is the output of the proportional controller
    double m_t_ratio; // this term is the fraction of maximum throttle required to balance the lander's mass
    double delta;
    

    

    altitude = position.abs() - MARS_RADIUS;
    descent_rate = velocity * position.norm(); // this is the term v . e_r
    m_t_ratio = g_force.abs() / MAX_THRUST;
    delta = K_d * m_t_ratio * K_p;

    // error term will be positive if (descent_rate - 0.5) is greater than K_h * altitude
    // K_h describes how strongly the altitude effects the error term

    e_term = - (0.5 + K_h * altitude + descent_rate);
    P_out = K_p * e_term;
    
    

    if (P_out < - delta) {
        throttle = 0;
    }
    else if (P_out < (1 - delta)) {
        throttle = delta + P_out;
    }
    else if (P_out > (1 - delta)) {
        throttle = 1;
    }
    
    cout << "Altitude: " << altitude << " Error term: " << e_term << " Throttle: " << throttle << endl;
   
    
    // parachute deployment condition taken directly from leander_graphics.cpp as the safe to deploy parachute boolean did not work
    double drag;

    // Assume high Reynolds number, quadratic drag = -0.5 * rho * v^2 * A * C_d
    drag = 0.5 * DRAG_COEF_CHUTE * atmospheric_density(position) * 5.0 * 2.0 * LANDER_SIZE * 2.0 * LANDER_SIZE * velocity.abs2();
    // Do not use the global variable "altitude" here, in case this function is called from within the
    // numerical_dynamics function, before altitude is updated in the update_visualization function
    if ((drag > MAX_PARACHUTE_DRAG) || ((velocity.abs() > MAX_PARACHUTE_SPEED) && ((position.abs() - MARS_RADIUS) < EXOSPHERE))) {
        parachute_status = NOT_DEPLOYED;
    }
    else if (P_out > -delta) {
        parachute_status = DEPLOYED;
    }

   

    // write hight and descent rate to their relevant lists
    h_list.push_back(altitude);
    ver_list.push_back(-descent_rate);

    if (altitude < LANDER_SIZE / 2.0) {
        // write heights and descent velocities to a file
        ofstream fout;
        fout.open("heights.txt");
        if (fout) { // file opened successfully
            for (int i = 0; i < h_list.size(); i = i + 1) {
                fout << h_list[i] << ' ' << ver_list[i] << ' ' << endl;
            }
            cout << "Successfully written to file" << endl;
        }
        else { // file did not open successfully
            cout << "Could not open file for writing" << endl;
        }
    }


 
}
void numerical_dynamics (void) {
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
  // INSERT YOUR CODE HERE

    // declare some variables
    double current_mass; // declare current mass variable 
    parachute_status_t parachute_status = NOT_DEPLOYED; // declare and initialise parachute status as not deployed
    double drag_coef;
    double EFF_AREA;
    double CA;

    // defining radius and altitude variables within function
    //these will need to be defined as global variables somehow later to be used in both functions
    double r;
    r = position.abs();

    // declare three main force vector variables for integration step
    vector3d thrust;
    /*vector3d g_force; */
    vector3d drag;

    // potentially put a while altitude > 0 in here

        //calculate current mass of lander
        double fuel_remaining;
        fuel_remaining = fuel * FUEL_CAPACITY * FUEL_DENSITY; // Calculate mass of remaining fuel
        current_mass = UNLOADED_LANDER_MASS + fuel_remaining;

        // calulate atmospheric density at position
        double at_density;
        at_density = atmospheric_density(position);

        // determine the product of the drag coefficient and effective area of lander module

        if (parachute_status == DEPLOYED) {
            CA = DRAG_COEF_LANDER * 3.14 * LANDER_SIZE * LANDER_SIZE + DRAG_COEF_CHUTE * 5 * (2 * LANDER_SIZE) * (2 * LANDER_SIZE);
        }
        else {
            drag_coef = DRAG_COEF_LANDER;
            EFF_AREA = 3.14 * LANDER_SIZE * LANDER_SIZE;
            CA = drag_coef * EFF_AREA;
        }
  
        // calculate the values of these variables
        r = position.abs(); // upadate value of radial distance
        g_force = (GRAVITY * MARS_MASS * current_mass *  (- 1) * position) / (r * r * r); // calculate gravitational force
        thrust = thrust_wrt_world(); // fetch thrust force
        drag = (- 0.5)* CA * at_density * velocity * velocity.abs(); // calculate drag force
  
        // use forces to update lander trajectory
        vector3d resultant_force;
        vector3d acceleration;
        resultant_force = g_force + thrust + drag;
        acceleration = resultant_force / current_mass;


        static vector3d previous_position;
        vector3d new_position;


        if (simulation_time < 0.001) {
            //euler update on first iteration
            velocity += acceleration * delta_t;
            position += velocity * delta_t;
            previous_position = position;
        }
        else {
            //verlet update
            new_position = (2 * position) - (previous_position) + (delta_t * delta_t * acceleration);
            velocity = (new_position - position) / delta_t;
            previous_position = position;
            position = new_position;
        }
        
      


  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
      position = vector3d(MARS_RADIUS + 500, 0.0, 0.0);
      velocity = vector3d(0.0, 0, 0.0);
      orientation = vector3d(0.0, 90.0, 0.0);
      delta_t = 0.1;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = false;
      autopilot_enabled = false;
      throttle = ((GRAVITY * MARS_MASS * 200) / ((MARS_RADIUS + 500) * (MARS_RADIUS + 500)))/MAX_THRUST;
    break;

  }
}
