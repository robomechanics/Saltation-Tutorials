# Saltation-Tutorials
This repository contains some starter code for the implementation of some basic saltation methods in MATLAB and python (for SKF only).

## Hybrid System Model Definitions
In this tutorial, we consider a variety of hybrid systems to demonstrate the Salted Kalman Filter (SKF) and Hybrid iLQR algorithms. Below we define these such systems:

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

## Tutorial Paper Code
This code provides an example of how to generate Figure 1 (below) from our paper [*Saltation Matrices: The Essential Tool for Linearizing Hybrid Dynamical Systems*](https://arxiv.org/abs/2306.06862). It includes implementations for 3 different toy systems: 
- a ball dropping on a slanted surface with frictionless sliding
- a ball dropping on a slanted surface with contrained motion due to static friction
- a bouncing ball on a slanted surface.
  
Example figures that can be generated with these files:

<img src="https://github.com/robomechanics/Saltation-Tutorials/blob/dev_dfriasfr/Tutorial%20Paper%20Code/plot_sticking.png" alt="Sticking" width="500">
<img src="https://github.com/robomechanics/Saltation-Tutorials/blob/dev_dfriasfr/Tutorial%20Paper%20Code/plot_sliding.png" alt="Sliding" width="500">

#### How to Generate the Saltation Matrix and Figures

1. **Run the `main.m` file** in the corresponding folder for the system you're interested in. This will calculate the saltation matrix and generate the plots.
   
2. **To calculate the saltation matrix for a new system:**
   - Modify the following files with your system's dynamics, guards, and resets:
     - `flows.m`
     - `guards.m`
     - `resets.m`
     - `saltation_calc.m`
   
3. After defining the new system, update the system initialization in the `main.m` file and rerun the code.
   
> **Note**: The saltation matrix is calculated symbolically in the dynamics files, so no further updates are needed for the matrix calculation. If you want to explore other systems or modify the existing ones, simply change the definitions in the respective files mentioned above. 

## Salted Kalman Filter
This code provides an example for implementing the SKF as defined in [*The Salted Kalman Filter: Kalman Filtering on Hybrid Dynamical Systems*](https://arxiv.org/abs/2007.12233).
### Models
In the MATLAB and python implementations, 2 different systems are considered: a simple 1D switching system and a 1D bouncing ball with a vertical thruster. Key parameters such as initial state `q0`, process noise `W`, and measurement noise `V` can be tuned to improve estimation accuracy. Examples of both filtered systems are shown below:

<img src="Salted%20Kalman%20Filter/simple_state_space.png" alt="Simple SKF" style="width:50%;">

<img src="Salted%20Kalman%20Filter/bouncing_ball_skf.png" alt="Bouncing Ball SKF" style="width:50%;">

### Python Structure

To run the examples:
1. Execute the corresponding Python file:
   - `simple_hybrid_system.py`
   - `bouncing_ball_hybrid_system.py`
2. Adjust parameters such as step size, noise covariances, etc., at the bottom of each script before running. :
   ```python
   simulate_timestep()
   ```

To simulate a different system's dynamics:
- Edit the expressions for the **flows**, **guards**, and **resets** in:
  ```python
  symbolic_dynamics()
  ```
- Re-run the file. No other changes are needed because:
  - The filter is automatically computed in `skf.py`.
  - The saltation matrix is calculated in `hybrid_helper_functions.py`.

### MATLAB Structure
To run, execute one of the scripts (e.g., `bouncing_ball_hybrid_system.m`) to visualize the SKF estimation results for the corresponding hybrid system. Example of output:
<img src="https://github.com/robomechanics/Saltation-Tutorials/blob/dev_dfriasfr/Salted%20Kalman%20Filter/Matlab/bouncing_ball_skf.png" alt="MATLAB SKF" width="500">

 
## Hybrid iLQR

This code provides an implementation of the **hybrid iLQR algorithm**, as defined in our paper [*iLQR for Piecewise-Smooth Hybrid Dynamical Systems*](https://arxiv.org/abs/2103.14584).

### Models Used for Trajectory Optimization

The example model used for trajectory optimization is a **1D bouncing ball**, as defined in the **SFK section** above. This module also includes the option to model a time-varying guard, such as a paddle moving up and down to bounce the ball. Below are some examples of optimized trajectories:


<img src="https://github.com/robomechanics/Saltation-Tutorials/blob/dev_dfriasfr/Hybrid%20iLQR/bouncing_ball_flat_guard_hilqr.png" alt="Flat Ground" width="500">
  
 
<img src="https://github.com/robomechanics/Saltation-Tutorials/blob/dev_dfriasfr/Hybrid%20iLQR/bouncing_ball_moving_guard_hilqr.png" alt="Moving Guard" width="500">

### Structure of the Code

To generate the optimized trajectories and plots:

1. **Run the `main.m` script**. This will perform the trajectory optimization for the 1D bouncing ball system.
   
2. **Symbolic Dynamics Calculation**: The symbolic dynamics are automatically computed by the `bouncing_dynamics()` function at the beginning of the script. No further modification is needed, apart from updating the initial states and control input guesses.

   - `bouncing_dynamics.m` uses the **MATLAB symbolic toolbox** to generate functions for:
     - Flows
     - Resets
     - Guards
     - Saltation matrix
     
   These generated functions are named: `calc_A.m`, `calc_f1.m`, `calc_salt12.m`, etc. **Do not modify these directly**.

3. **Modifying or Modeling a New System**:  
   - To model a new system or modify the existing one, you need to update the expressions in the `bouncing_dynamics.m` file and rerun the optimization in `main.m`.

4. **Trajectory Optimization**:  
   - The optimization is performed using the `hilqr.m` class. This class handles the forwards/backwards passes of the hybrid iLQR algorithm. 
   - **Do not modify** this file directly, as it relies on a generalized optimization structure input from the `main.m` script.
  
---

### Additional Notes

- Ensure you have MATLAB installed to run these scripts!

