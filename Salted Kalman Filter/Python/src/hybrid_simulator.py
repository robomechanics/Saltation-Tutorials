
import numpy as np
from scipy.integrate import solve_ivp
from src.hybrid_helper_functions import (
    solve_ivp_dynamics_func,
    solve_ivp_guard_funcs,
    solve_ivp_extract_hybrid_events,
)

class HybridSimulator:
    def __init__(self,init_state,init_mode,dt,noise_matrices,dynamics,resets, guards, parameters):
        """
        init_state (np.array): Initial state.
        noise_matrices (np.array): Noise matrices for each mode.
        dynamics (dict): Dynamics for each mode.
        resets (dict): Resets for each allowable transition.
        guards (dict): Guards for each allowable transition.
        parameters (np.array): Extra parameters of the system.
        """
        self._current_state = init_state
        self._current_mode = init_mode
        self._dt = dt
        self._dynamics_dict = dynamics
        self._resets_dict = resets
        self._guards_dict = guards
        self._parameters = parameters
        self._noise_matrices = noise_matrices
        self._n_states = np.shape(self._current_state)[0]
        


    def simulate_timestep(self, current_time, inputs):
        """
        Simulates for one dt.
        """
        end_time = current_time + self._dt

        """ Get noise parameters. """
        process_gaussian_noise = {
            "mean": np.zeros(self._n_states),
            "cov": self._noise_matrices[self._current_mode]['W']
        }
        """ Integrate for dt. """
        current_dynamics = solve_ivp_dynamics_func(
            self._dynamics_dict, self._current_mode, inputs, self._dt, self._parameters, process_gaussian_noise=process_gaussian_noise
        )
        current_guards, possible_modes = solve_ivp_guard_funcs(
            self._guards_dict, self._current_mode, inputs, self._dt, self._parameters
        )

        sol = solve_ivp(
            current_dynamics,
            [current_time, end_time],
            self._current_state,
            events=current_guards,
        )
        current_state = np.zeros(self._n_states)

        """ If we hit guard, apply reset. """
        (
            hybrid_event_state,
            hybrid_event_time,
            new_mode,
        ) = solve_ivp_extract_hybrid_events(sol, possible_modes)

        while new_mode is not None:
            """Apply reset."""
            current_state = self._resets_dict[self._current_mode][new_mode]['r'](
                hybrid_event_state, inputs, self._dt, self._parameters
            ).reshape(np.shape(hybrid_event_state))

            """ Update guard and simulate. """
            self._current_mode = new_mode
            current_dynamics = solve_ivp_dynamics_func(
                self._dynamics_dict,
                self._current_mode,
                inputs,
                self._dt,
                self._parameters,
            )
            current_guards, possible_modes = solve_ivp_guard_funcs(
                self._guards_dict,
                self._current_mode,
                inputs,
                self._dt,
                self._parameters,
            )
            sol = solve_ivp(
                current_dynamics,
                [hybrid_event_time, end_time],
                current_state,
                events=current_guards,
            )
            (
                hybrid_event_state,
                hybrid_event_time,
                new_mode,
            ) = solve_ivp_extract_hybrid_events(sol, possible_modes)

        """ Once no more hybrid events, grab the terminal states. """
        for idx in range(len(sol.y)):
            """Grab the state at the last timestep."""
            current_state[idx] = sol.y[idx][-1]

        self._current_state = current_state

    def get_measurement(self, measurement_noise_flag = False):
        measurement = self._dynamics_dict[self._current_mode]['y'](
                self._current_state,
                self._parameters,
            ).flatten()
        if not measurement_noise_flag:
            return measurement
        else:
            measurement_gaussian_noise = {
            "mean": np.zeros(np.shape(measurement)[0]),
            "cov": self._noise_matrices[self._current_mode]['V']
        }
            return measurement + np.random.multivariate_normal(measurement_gaussian_noise["mean"], measurement_gaussian_noise["cov"])
    
    def get_state(self):
        return self._current_state.copy()