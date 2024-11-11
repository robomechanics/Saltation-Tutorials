import sys
import pathlib
import numpy as np
from scipy.integrate import solve_ivp

sys.path.append(str(pathlib.Path(__file__).parent.parent))
from src.hybrid_helper_functions import (
    solve_ivp_dynamics_func,
    solve_ivp_guard_funcs,
    solve_ivp_extract_hybrid_events,
    compute_saltation_matrix,
)

class SKF:
    def __init__(
        self,
        init_state,
        init_mode,
        init_cov,
        dt,
        noise_matrices,
        dynamics,
        resets,
        guards,
        parameters,
    ):
        """
        init_state (np.array): Initial state.
        init_cov (np.array): Initial covariance.
        noise_matrices (np.array): Noise matrices for each mode.
        dynamics (dict): Dynamics for each mode.
        resets (dict): Resets for each allowable transition.
        guards (dict): Guards for each allowable transition.
        parameters (np.array): Extra parameters of the system.
        """
        self._current_state = init_state
        self._current_cov = init_cov
        self._current_mode = init_mode
        self._dt = dt
        self._noise_matrices_dict = noise_matrices
        self._dynamics_dict = dynamics
        self._resets_dict = resets
        self._guards_dict = guards
        self._parameters = parameters

        self._n_states = np.shape(self._current_state)[0]

    def predict(self, current_time, inputs):
        """
        Prior update.
        """
        end_time = current_time + self._dt

        """ Integrate for dt. """
        current_dynamics = solve_ivp_dynamics_func(
            self._dynamics_dict, self._current_mode, inputs, self._dt, self._parameters
        )
        current_guards, possible_modes = solve_ivp_guard_funcs(
            self._guards_dict, self._current_mode, inputs, self._dt, self._parameters
        )

        current_start_state = self._current_state.copy()
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

            """ Apply covairance updates: dynamics and saltation matrix."""
            dynamics_cov = self._dynamics_dict[self._current_mode]["A_disc"](
                current_start_state, inputs, sol.t[-1] - sol.t[0], self._parameters
            )
            self._current_cov = (
                dynamics_cov @ self._current_cov @ dynamics_cov.T
                + self._noise_matrices_dict[self._current_mode]["W"]
            )
            salt = compute_saltation_matrix(
                t=hybrid_event_time,
                pre_event_state=hybrid_event_state,
                inputs=inputs,
                dt=self._dt,
                parameters=self._parameters,
                pre_mode=self._current_mode,
                post_mode=new_mode,
                dynamics_dict=self._dynamics_dict,
                resets_dict=self._resets_dict,
                guards_dict=self._guards_dict,
                post_event_state=current_state,
            )
            self._current_cov = salt @ self._current_cov @ salt.T

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
            current_start_state = current_state.copy()
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

        """ Propagate the rest of the covariance. """
        dynamics_cov = self._dynamics_dict[self._current_mode]["A_disc"](
            current_start_state, inputs, sol.t[-1] - sol.t[0], self._parameters
        )

        print(current_time)
        
        self._current_cov = (
            dynamics_cov @ self._current_cov @ dynamics_cov.T
            + self._noise_matrices_dict[self._current_mode]["W"]
        )

        self._current_state = current_state
        return self._current_state, self._current_cov

    def update(self, current_time, current_input, measurement):
        """
        Posterior update.
        When a new measurement comes in, update the covariance.
        If updated state is pulled into new mode, then apply saltation matrix and reset.
        """
        C = self._dynamics_dict[self._current_mode]['C'](
                self._current_state,
                self._parameters,
            )
        V = self._noise_matrices_dict[self._current_mode]['V']
        K = self._current_cov@C.T@np.linalg.inv(C@self._current_cov@C.T + V)

        """ Measurement update. """
        measurement_est = self._dynamics_dict[self._current_mode]['y'](
                self._current_state,
                self._parameters,
            ).flatten()
        C = self._dynamics_dict[self._current_mode]['C'](
                self._current_state,
                self._parameters,
            )
        residual = measurement - measurement_est
        self._current_state = self._current_state + K@residual
        self._current_cov = self._current_cov - K@C@self._current_cov

        """ Check guard conditions. If any guard is has been reached, then apply hybrid posterior update. """
        current_guards, possible_modes = solve_ivp_guard_funcs(
            self._guards_dict, self._current_mode, current_input, self._dt, self._parameters
        )
        for guard_idx in range(len(current_guards)):
            """TODO: Add in time component. """
            if current_guards[guard_idx](current_time, self._current_state) < 0:
                print("Debug: Applying posterior hybrid update.")
                new_mode = possible_modes[guard_idx]
                """Apply reset."""
                new_state = self._resets_dict[self._current_mode][new_mode]['r'](
                    self._current_state, current_input, self._dt, self._parameters
                ).reshape(np.shape(self._current_state))

                """ Apply covairance updates: dynamics and saltation matrix."""
                salt = compute_saltation_matrix(
                    t=current_time,
                    pre_event_state=self._current_state,
                    inputs=current_input,
                    dt=self._dt,
                    parameters=self._parameters,
                    pre_mode=self._current_mode,
                    post_mode=new_mode,
                    dynamics_dict=self._dynamics_dict,
                    resets_dict=self._resets_dict,
                    guards_dict=self._guards_dict,
                    post_event_state=new_state,
                )
                self._current_state = new_state
                self._current_cov = salt @ self._current_cov @ salt.T
                break

        return self._current_state, self._current_cov
