# Saltation-Tutorials
This repository contains some starter code for the implementation of some basic saltation methods in MATLAB. 
## Tutorial Paper Code
This code provides an example of how to generate Figure 1 (below) from our paper [Saltation Matrices: The Essential Tool for Linearizing Hybrid Dynamical Systems](https://arxiv.org/abs/2306.06862). It includes implementations for 3 different toy systems: a ball dropping on a slanted surface with frictionless sliding, a ball dropping on a slanted surface with contrained motion due to static friction, and a bouncing ball on a slanted surface. 
## Salted Kalman Filter
This code provides an example for implementing the SKF as defined in [The Salted Kalman Filter: Kalman Filtering on Hybrid Dynamical Systems](https://arxiv.org/abs/2007.12233).
### Models
In the python implementation, 3 different systems are considered: 
### Structure
 
## Hybrid iLQR
This code provides an example for implementing the hybrid iLQR algoirthm as defined in [iLQR for Piecewise-Smooth Hybrid Dynamical Systems](https://arxiv.org/abs/2103.14584).
### Models
The model used for the trajectory optimzation is a 1D bouncing ball, as defined in the SFK section above.
### Structure
Run main2.m (or rather, I need to go back and delete main.m since it doesn't generalize w/ the moving guard dynamics :/ )
