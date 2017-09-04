# Model Predictive Control Poject
Udacity Self-Driving Car Engineer Nanodegree Program

Term 2, Project 5

---

## Objective
**The objective of this project is to implement a Model Predictive Control (MPC) to drive a car around a simulated track.**
In the model, the cross track error is calculated and a latency of 100 milliseconds is accounted for.

## Implementation

./My Reflection on MPC.pdf discusses the implmentaton details and how to run the mpc with different hyperparams

## Directory structure

	./src
	./build
	./experiments
			run1
			run2
			run3
			run4
	./My Reflection on MPC.pdf

## Run

	go to ./src directory
	review run.sh, change the parms if needed and run ./run.sh
	./experiments directory contains various experiments during optimization process. Review run.sh of various experiments. The results are documented in My Reflecton o MPC PDF

### MPC with Latency
Latency of `100ms` was accounted for in the global kinematic model implementation. A `dt` of `0.05` was chosen. It was observed that the model performed better with the global kinematic model implementation.

## 2.0 Simulation
Follow the link below to see a video of the result of the MPC implementation in the simulator.

### Summary
Once the model has been implemented to correctly update variables and actuators, the weights on each part of the cost function can be tuned to give a better performance. You can modify run.sh with different params and run the experiment. This had an impact on how fast or how smoothly the car could drive. After tuning these weights, the vehicle is able to drive safely up to a speed of 80 mph.
