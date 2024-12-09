# Saltation-Tutorials
This repository contains some starter code for the implementation of some basic saltation methods in MATLAB and python (for SKF only).

## Tutorial Paper Code
This code provides an example of how to generate Figure 1 (below) from our paper [Saltation Matrices: The Essential Tool for Linearizing Hybrid Dynamical Systems](https://arxiv.org/abs/2306.06862). It includes implementations for 3 different toy systems: a ball dropping on a slanted surface with frictionless sliding, a ball dropping on a slanted surface with contrained motion due to static friction, and a bouncing ball on a slanted surface. 

Example figures that can be generated with these files:
![Sticking](https://github.com/robomechanics/Saltation-Tutorials/blob/dev_dfriasfr/Tutorial%20Paper%20Code/plot_sticking.png)
![Sliding](https://github.com/robomechanics/Saltation-Tutorials/blob/dev_dfriasfr/Tutorial%20Paper%20Code/plot_sliding.png)

To calculate the saltation and generate these plots, simply run "main.m" in the corrensponding folder of the system your're interested in. If you wish to calculate the saltation matrix for a different system, simply define your new dynamics/guards/resets in the correspodning fileds "flows.m", "guards.m", "resets.m" and "saltation_calc.m" and rerun after updating the system initializaiton in "main.m". The saltation matrix is calculated symbolically in the dynamics files, so no futher update should be needed.  

## Salted Kalman Filter
This code provides an example for implementing the SKF as defined in [The Salted Kalman Filter: Kalman Filtering on Hybrid Dynamical Systems](https://arxiv.org/abs/2007.12233).
### Models
In the python implementation, 2 different systems are considered: a simple 1D switching system and a 1D bouncing ball with a vertical thruster. Examples of both filtered systems are shown below:

<img src="Salted%20Kalman%20Filter/simple_state_space.png" alt="Simple SKF" style="width:50%;">

<img src="Salted%20Kalman%20Filter/bouncing_ball_skf.png" alt="Bouncing Ball SKF" style="width:50%;">

### Structure

To run the examples:
1. Execute the corresponding Python file:
   - `simple_hybrid_system.py`
   - `bouncing_ball_hybrid_system.py`
2. Adjust parameters such as step size, noise covariances, etc., at the bottom of each script before running:
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

 
## Hybrid iLQR
This code provides an example for implementing the hybrid iLQR algoirthm as defined in [iLQR for Piecewise-Smooth Hybrid Dynamical Systems](https://arxiv.org/abs/2103.14584).
### Models
The model used for the trajectory optimzation is a 1D bouncing ball, as defined in the SFK section above.
### Structure
Run main2.m (or rather, I need to go back and delete main.m since it doesn't generalize w/ the moving guard dynamics :/ )
