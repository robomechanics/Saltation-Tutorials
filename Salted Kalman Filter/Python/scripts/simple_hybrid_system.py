"""
simple_hybrid_system.py

This script simulates a simple 2D hybrid dynamical system using a Salted Kalman Filter (SKF) for hybrid state estimation.
The system switches between two modes:
- Mode 'I': constant dynamics with a negative flow component
- Mode 'J': constant dynamics with a positive flow component

A guard triggers a reset when the first state variable crosses zero.
The SKF tracks the system state despite noisy measurements and switches modes upon crossing the guard.

Key Components:
- Defines symbolic continuous dynamics, measurements, resets, and guard conditions.
- Constructs noise models for process and measurement noise.
- Initializes a Salted Kalman Filter (SKF) using the symbolic hybrid model.
- Simulates the hybrid system with process noise and measurement noise.
- Performs SKF-based state estimation across hybrid transitions.
- Visualizes:
    - Actual system trajectory
    - Noisy measurements
    - Estimated trajectory (SKF output)
"""

import sys
import pathlib
import time
import numpy as np
import sympy as sp
from sympy.matrices import Matrix
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

sys.path.append(str(pathlib.Path(__file__).parent.parent))
from src.skf import SKF
from src.hybrid_simulator import HybridSimulator

def symbolic_dynamics():
    """
    Returns (Tuple[Dict, Dict]): dynamic functions in a nested dict and reset functions in a nested dict.
    """
    x1, x2, u, dt, t = sp.symbols("x1 x2 u dt t")

    """ Define the states and inputs. """
    inputs = Matrix([u])  # note: inputs are included for generality but are unused in this system
    states = Matrix([x1, x2])
    time = Matrix([t])

    """ Defining the dynamics of the system. """
    fI = Matrix([1, -1])
    fJ = Matrix([1, 1])
    
    """ Define the measurements of the system. """
    yI = Matrix([x1, x2])
    yJ = Matrix([x1, x2])

    """ Discretize the dynamics using euler integration. """
    fI_disc = states + fI * dt
    fJ_disc = states + fJ * dt

    """ Take the jacobian with respect to states and inputs. """
    AI_disc = fI_disc.jacobian(states)
    AJ_disc = fJ_disc.jacobian(states)

    """ Take the jacobian of the measurements with respect to the states. """
    CI = yI.jacobian(states)
    CJ = yJ.jacobian(states)

    """ Define resets. """
    rIJ = Matrix([x1, x2])

    """ Take the jacobian of resets with resepct to states. """
    RIJ = rIJ.jacobian(states)

    """ Define guards. """
    gIJ = Matrix([-x1])

    """ Take the jacobian of resets with resepct to guards. """
    GIJ = gIJ.jacobian(states)
    GtIJ = gIJ.jacobian(time)

    """ Define the parameters of the system. """
    parameters = Matrix([])

    rIJ_func = sp.lambdify((states, inputs, dt, parameters), rIJ)
    RIJ_func = sp.lambdify((states, inputs, dt, parameters), RIJ)

    gIJ_func = sp.lambdify((t, states, inputs, dt, parameters), gIJ)
    GIJ_func = sp.lambdify((states, inputs, dt, parameters), GIJ)
    GtIJ_func = sp.lambdify((t, states, inputs, dt, parameters), GtIJ)

    fI_func = sp.lambdify((states, inputs, dt, parameters), fI)
    AI_disc_func = sp.lambdify((states, inputs, dt, parameters), AI_disc)

    fJ_func = sp.lambdify((states, inputs, dt, parameters), fJ)
    AJ_disc_func = sp.lambdify((states, inputs, dt, parameters), AJ_disc)

    yI_func = sp.lambdify((states, parameters), yI)
    CI_func = sp.lambdify((states, parameters), CI)

    yJ_func = sp.lambdify((states, parameters), yJ)
    CJ_func = sp.lambdify((states, parameters), CJ)

    dynamics = {
        "I": {"f_cont": fI_func, "A_disc": AI_disc_func, "y": yI_func, "C": CI_func},
        "J": {"f_cont": fJ_func, "A_disc": AJ_disc_func, "y": yJ_func, "C": CJ_func},
    }
    resets = {"I": {"J": {"r": rIJ_func, "R": RIJ_func}}}
    guards = {"I": {"J": {"g": gIJ_func, "G": GIJ_func, "Gt": GtIJ_func}}}
    return dynamics, resets, guards


""" Define dynamics and resets. """
dynamics, resets, guards = symbolic_dynamics()

""" Define noise matrices. """
n_states = 2
W_global = 0.01 * np.eye(n_states)
V_global = 0.025 * np.eye(n_states)
noise_matrices = {
    "I": {"W": W_global, "V": V_global},
    "J": {"W": W_global, "V": V_global},
}

""" Initialize states and covariance. """
mean_init_state = np.array([-2.5, 0])
mean_init_cov = 0.1*np.eye(n_states)
init_mode = "I"  # Modes are {I, J}

""" Define timesteps. """
dt = 0.1

""" Define parameters. """
parameters = np.array([])

""" Initialize filter. """
skf = SKF(
    init_state=mean_init_state,
    init_mode=init_mode,
    init_cov=mean_init_cov,
    dt=dt,
    noise_matrices=noise_matrices,
    dynamics=dynamics,
    resets=resets,
    guards=guards,
    parameters=parameters
)

""" Initialize simulator. """
actual_init_state = np.random.multivariate_normal(mean_init_state,mean_init_cov)
hybrid_simulator = HybridSimulator(
    init_state=actual_init_state,
    init_mode=init_mode,
    dt=dt,
    noise_matrices=noise_matrices,
    dynamics=dynamics,
    resets=resets,
    guards=guards,
    parameters=parameters
)

""" Run SKF simulation """

n_simulate_timesteps = 50
timesteps = np.arange(0.0,n_simulate_timesteps*dt,dt)
measurements = np.zeros((n_simulate_timesteps-1,n_states))
actual_states = np.zeros((n_simulate_timesteps,n_states))
filtered_states = np.zeros((n_simulate_timesteps,n_states))

actual_states[0,:] = hybrid_simulator.get_state()
filtered_states[0,:] = mean_init_state

zero_input = np.array([0.0])
for time_idx in range(1,n_simulate_timesteps):
    hybrid_simulator.simulate_timestep(0,np.array([0]))
    actual_states[time_idx,:] = hybrid_simulator.get_state()
    measurements[time_idx-1,:] = hybrid_simulator.get_measurement(measurement_noise_flag=True)
    skf.predict(timesteps[time_idx],zero_input)
    filtered_states[time_idx,:], current_cov = skf.update(timesteps[time_idx],zero_input,measurements[time_idx-1,:])

""" Visualize results """

plt.plot(actual_states[:,0],actual_states[:,1],'k-',label='Actual states')
plt.plot(measurements[:,0],measurements[:,1],'r.',label='Measurements')
plt.plot(filtered_states[:,0], filtered_states[:,1],'b--',label='Filtered states')
plt.legend()
plt.xlabel(r"$y$")
plt.ylabel(r"$\dot{y}$")
plt.title("1D Bouncing Ball System")
plt.show()
