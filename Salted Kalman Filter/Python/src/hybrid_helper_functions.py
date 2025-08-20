"""
hybrid_helper_functions.py

This file provides utility functions for simulating hybrid dynamical systems using
SciPy's `solve_ivp` integrator. It includes support for continuous integration, guard detection,
mode switching, and saltation matrix computation for hybrid systems.

Key Components:
- `solve_ivp_dynamics_func`: Wraps a continuous dynamics function into a `solve_ivp`-compatible format,
  optionally adding Gaussian process noise.
- `solve_ivp_guard_funcs`: Wraps hybrid guards into `solve_ivp` event functions for detecting mode transitions.
- `solve_ivp_extract_hybrid_events`: Extracts hybrid events (mode switches) from a completed `solve_ivp` simulation.
- `compute_saltation_matrix`: Computes the saltation matrix used to propagate state uncertainty across
  hybrid transitions (discontinuities).
"""

import numpy as np

def solve_ivp_dynamics_func(dynamics_dict, mode, inputs, dt, parameters, process_gaussian_noise = None):
    """
    Create a lambda from our dynamics which works with solve_ivp.
    """
    if process_gaussian_noise is not None:
        process_noise = np.random.multivariate_normal(process_gaussian_noise["mean"], process_gaussian_noise["cov"])
        return lambda t, states: dynamics_dict[mode]["f_cont"](
            states, inputs, dt, parameters
        ).reshape(np.shape(states)) + process_noise
    else:
        return lambda t, states: dynamics_dict[mode]["f_cont"](
            states, inputs, dt, parameters
        ).reshape(np.shape(states))


def solve_ivp_guard_funcs(guards_dict, mode, inputs, dt, parameters):
    """
    Create a lambda from our guards which works with solve_ivp.
    """
    guards = []
    new_modes = []
    if mode in guards_dict:
        for key, val in guards_dict[mode].items():
            guard = lambda t, states: val["g"](t, states, inputs, dt, parameters)
            guard.terminal = True
            guard.direction = -1
            guards.append(guard)
            new_modes.append(key)
    return guards, new_modes


def solve_ivp_extract_hybrid_events(sol, possible_modes):
    """
    Extracts the hybrid events during a solve_ivp solve.
    """
    for idx in range(len(possible_modes)):
        """Assume we cannot activate multiple guards at once."""
        if sol.t_events[idx]:
            return sol.y_events[idx].flatten(), sol.t_events[idx], possible_modes[idx] # Flatten used here for compatibility — could replace with reshape if needed.
    return None, None, None


def compute_saltation_matrix(
    t,
    pre_event_state,
    inputs,
    dt,
    parameters,
    pre_mode,
    post_mode,
    dynamics_dict,
    resets_dict,
    guards_dict,
    post_event_state=None,
):
    """
    Computes the saltation matrix.
    Note: Saltation matrix currently assumes time-invariant reset and guard maps.
    """
    if post_event_state is None:
        """ Compute reset if not post event state is given. """
        post_event_state = resets_dict[pre_mode][post_mode]['r'](
            pre_event_state, inputs, dt, parameters
        ).reshape(np.shape(pre_event_state))
    
    DxR = resets_dict[pre_mode][post_mode]['R'](
        pre_event_state, inputs, dt, parameters
    )
    DxG = guards_dict[pre_mode][post_mode]['G'](
        pre_event_state, inputs, dt, parameters
    )
    DtG = guards_dict[pre_mode][post_mode]['Gt'](
        t, pre_event_state, inputs, dt, parameters
    )
    f_pre = dynamics_dict[pre_mode]['f_cont'](
        pre_event_state, inputs, dt, parameters
    ).reshape(np.shape(pre_event_state))
    f_post = dynamics_dict[post_mode]['f_cont'](
        post_event_state, inputs, dt, parameters
    ).reshape(np.shape(pre_event_state))

    salt = DxR + np.outer((f_post - DxR@f_pre),DxG)/(DtG + DxG@f_pre)
    return salt
