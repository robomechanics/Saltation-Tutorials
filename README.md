# Saltation-Tutorials
This repository contains some starter code for the implementation of some basic saltation methods in MATLAB. 
## Tutorial Paper Code
This code provides an example of how to generate Figure 1 (below) from our paper [Saltation Matrices: The Essential Tool for Linearizing Hybrid Dynamical Systems](https://arxiv.org/abs/2306.06862). It includes implementations for 3 different toy systems: 
- a ball dropping on a slanted surface with frictionless sliding
- a ball dropping on a slanted surface with contrained motion due to static friction
- a bouncing ball on a slanted surface. 

## Hybrid System Models
In addition, we implement a variety of hybrid systems to demonstrate the Salted Kalman Filter (SKF) and Hybrid iLQR algorithms. Below are the systems used in the `Salted Kalman Filter` implementation:

### 1. Simple Hybrid System
- **Dynamics**: Two modes, `I` and `J`, with dynamics:
  - Mode `I`: `dx/dt = [1; -1]`
  - Mode `J`: `dx/dt = [1; 1]`
- **Guards**:
  - `g_{I → J}: x1 = 0` (transition from `I` to `J` occurs when `x1` crosses zero).

- **Resets**: 
  - `r_{I → J}: [x1; x2] → [x1; x2]` (identity reset for transition from `I` to `J`).

### 2. Bouncing Ball Hybrid System

- **Dynamics**: Two modes, `I` and `J`, with identical dynamics:
  - Mode `I` and Mode `J`: `dx/dt = [q_dot; -g]`, where `g` is gravitational acceleration.
- **Guards**: 
  - `g_{I → J}: q - 0 = 0` (ball impacts the ground).
  - `g_{J → I}: q_dot = 0`.
- **Resets**:
  - `r_{I → J}: [q; q_dot] → [q; -e * q_dot]`, where `e` is the coefficient of restitution.
  - `r_{J → I}: [q; q_dot] → [q; q_dot]` (identity reset).

  ### 3. Bouncing Ball with Moving Guard Hybrid System

- **Dynamics**: Two modes, `I` and `J`, with identical dynamics:
  - Mode `I` and `J`: `dx/dt = [q_dot; -g]`, where `g` is gravitational acceleration.

- **Guards**:
  - `g_{I → J}: q = 0.25 * sin(4πt)` (impact with a moving paddle).
  - `g_{J → I}: q_dot = 0`.

- **Resets**:
  - `r_{I → J}: [q; q_dot] → [q; -e * q_dot]`, where `e` is the coefficient of restitution.
  - `r_{J → I}: [q; q_dot] → [q; q_dot]` (identity reset).


## Salted Kalman Filter
This code provides an example for implementing the SKF as defined in [The Salted Kalman Filter: Kalman Filtering on Hybrid Dynamical Systems](https://arxiv.org/abs/2007.12233).

Key parameters such as initial state `q0`, process noise `W`, measurement noise `V`, guards, and resets can be tuned to improve estimation accuracy.

### Models
The code includes implementations in MATLAB and Python for the simple hybrid system, bouncing ball, and bouncing ball with moving guard (defined in the Hybrid System Models section above).

### Structure
#### MATLAB / Python
To run, execute one of the scripts (e.g., `bouncing_ball_hybrid_system.m`) to visualize the SKF estimation results for the corresponding hybrid system.

## Hybrid iLQR
This code provides an example for implementing the hybrid iLQR algoirthm as defined in [iLQR for Piecewise-Smooth Hybrid Dynamical Systems](https://arxiv.org/abs/2103.14584).
### Models
The model used for the trajectory optimzation is a 1D bouncing ball, as defined in the SFK section above.
### Structure
Run main2.m (or rather, I need to go back and delete main.m since it doesn't generalize w/ the moving guard dynamics :/ )
