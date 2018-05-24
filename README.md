# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

#Introduction

Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass which
reduces the accuracy of the models, but it also makes them more tractable. 
At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics. So dynamic models aim to embody the actual vehicle dynamics as closely as possible.
They might encompass tire forces, longitudinal and lateral forces, inertia, gravity, air resistance, drag, mass, and the geometry of the vehicle.

In this project we are trying to implement a Kinematic model. This repository includes the code implemetation of MPC for carnd term2 project.
#The Model

States:-

  I have assumed the variables as follows:

px - is the x-coordinate of current location of vehicle.
py - is y-coordinate of current location of vehicle.
psi - is the current orientation of vehicle.
v - current velocity of the vehicle.
cte - is cross track error, which is the distance of the current trajectory of the vehicle w.r.t desired trajectory of the vehicle.
epsi - is the error in orientation of vehicle corresponding to desired orientation.


Actuations:-

delta - steering angle of the vehicle. This angle is restricted between 25 to -25 degrees (as per the guidelines given in the classroom as well as the code snippets).
a - throttle  of the vehicle which accelerates or retards the vehicle. Value of throttle is restricted between -1 to 1.


Kinematic Model:-

The kinematic model equations define the state variables after time t and the expressions are given as:

px_t = px + v * cos(psi) *dt
py_t = py + v * sin(psi) *dt
psi_t = psi + v / Lf * -delta *dt
v_t = v + a * dt
cte_t = cte + v * sin(epsi) * dt
espi_t = espi + v / Lf * -delta *dt

where a is the throttle, and Lf measures the distance between the center of mass of the vehicle and it's front axle. The larger the vehicle, the slower the turn rate. The value of Lf is chosen to be constant value of 2.67.


N & dt:

I have used the values for N and dt as 25 and 0.05 respectively. I have reffered to The Q&A sessions for reffering to the values. The value of N (timestamp length) implies the number of future steps we want to predict from the current location of the car and dt is the duration.

MPC Preprocessing:

In MPC preprocessing the before passing values to the polyfit function of degree 3 a conversion of thpse values to vehicle co-ordinate system is done which results in easier calculations of the vars, as the vehicle location will be treated to origin with x and y as 0. The co-ordinates are shifted so that the are aligned with the vehicle co-ordinate system.Using a 3rd dergee polynomial instead of taking a higher order polynimial results in higher computatinal work and lower order polynomial won't give us proper approximation.


Model Predictive Control with Latency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.
This is a problem called "latency", and it's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.

To overcome latency in the code I have usied Kinematic model equations specified in the code to compute the state. Then these predicted states are passed to the model for optimizing.
