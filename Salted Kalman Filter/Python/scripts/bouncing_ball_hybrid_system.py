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
    TODO: FILL IN WITH BOUNCING BALL. Modes are {'up','down'}. e is coefficient of resititution.
    """
    q, q_dot, e, g, u, dt, t = sp.symbols("q q_dot e g u dt t")

    """ Define the states and inputs. """
    inputs = Matrix([u])
    states = Matrix([q, q_dot])
    time = Matrix([t])

    """ FILL IN EVERYTHING ELSE BELOW HERE!! """
    """ Defining the dynamics of the system. """
    fI = Matrix([q_dot, -g])
    fJ = Matrix([q_dot, -g])
    
    """ Define the measurements of the system. """
    yI = Matrix([q, q_dot])
    yJ = Matrix([q, q_dot])

    """ Discretize the dynamics usp.sing euler integration. """
    fI_disc = states + fI * dt
    fJ_disc = states + fI * dt

    """ Take the jacobian with respect to states and inputs. """
    AI_disc = fI_disc.jacobian(states)
    AJ_disc = fJ_disc.jacobian(states)

    """ Take the jacobian of the measurements with respect to the states. """
    CI = yI.jacobian(states)
    CJ = yJ.jacobian(states)

    """ Define resets. """
    rIJ = Matrix([q, -e*q_dot])
    rJI = Matrix([q, q_dot])

    """ Take the jacobian of resets with resepct to states. """
    RIJ = rIJ.jacobian(states)
    RJI = rJI.jacobian(states)

    """ Define guards. """
    x_p = 0 # guard is located at y = 0
    gIJ = Matrix([q - x_p])
    gJI = Matrix([q_dot])

    """ Take the jacobian of resets with resepct to guards. """
    GIJ = gIJ.jacobian(states)
    GJI = gJI.jacobian(states)

    """ Take the jacobian of guards w/ respect to time"""
    GtIJ = gIJ.jacobian(time)
    GtJI = gJI.jacobian(time)


    """ Define the parameters of the system. """
    parameters = Matrix([e, g])

    rIJ_func = sp.lambdify((states, inputs, dt, parameters), rIJ)
    RIJ_func = sp.lambdify((states, inputs, dt, parameters), RIJ)

    rJI_func = sp.lambdify((states, inputs, dt, parameters), rJI)
    RJI_func = sp.lambdify((states, inputs, dt, parameters), RJI)

    gIJ_func = sp.lambdify((t, states, inputs, dt, parameters), gIJ)
    GIJ_func = sp.lambdify((states, inputs, dt, parameters), GIJ)
    GtIJ_func = sp.lambdify((t, states, inputs, dt, parameters), GtIJ)

    gJI_func = sp.lambdify((t, states, inputs, dt, parameters), gJI)
    GJI_func = sp.lambdify((states, inputs, dt, parameters), GJI)
    GtJI_func = sp.lambdify((t, states, inputs, dt, parameters), GtJI)

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
    resets = {"I": {"J": {"r": rIJ_func, "R": RIJ_func}}, "J": {"I": {"r": rJI_func, "R": RJI_func}}}
    guards = {"I": {"J": {"g": gIJ_func, "G": GIJ_func, "Gt": GtIJ_func}}, "J": {"I": {"g": gJI_func, "G": GJI_func, "Gt": GtJI_func}}}
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
mean_init_state = np.array([5, 0])
mean_init_cov = 0.1*np.eye(n_states)
init_mode = "I"  # Modes are {I, J}

""" Define timesteps. """
dt = 0.05

""" Define parameters. """
parameters = np.array([0.7, 9.8]) # [coeff of rest., gravity, mass]

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

n_simulate_timesteps = 100
timesteps = np.arange(0.0,n_simulate_timesteps*dt,dt)
measurements = np.zeros((n_simulate_timesteps-1,n_states))
actual_states = np.zeros((n_simulate_timesteps,n_states))
filtered_states = np.zeros((n_simulate_timesteps,n_states))
# guard = 0.25*np.sin(4*np.pi*timesteps*dt)
guard = 0.0*timesteps

actual_states[0,:] = hybrid_simulator.get_state()
filtered_states[0,:] = mean_init_state

zero_input = np.array([0.0])
for time_idx in range(1,n_simulate_timesteps):
    hybrid_simulator.simulate_timestep(0,np.array([0]))
    actual_states[time_idx,:] = hybrid_simulator.get_state()
    measurements[time_idx-1,:] = hybrid_simulator.get_measurement(measurement_noise_flag=True)
    skf.predict(timesteps[time_idx],zero_input)
    filtered_states[time_idx,:], current_cov = skf.update(timesteps[time_idx],zero_input,measurements[time_idx-1,:])

# plt.plot(actual_states[:,0],actual_states[:,1],'k-',label='Actual states')
# plt.plot(measurements[:,0],measurements[:,1],'r.',label='Measurements')
# plt.plot(filtered_states[:,0], filtered_states[:,1],'b--',label='Filtered states')
# plt.legend()
# plt.xlabel(r"$y$")
# plt.ylabel(r"$\dot{y}$")
# plt.title("1D Bouncing Ball System")
# plt.show()

plt.plot(actual_states[:,0],'k-',label='Actual states')
plt.plot(measurements[:,0],'r.',label='Measurements')
plt.plot(filtered_states[:,0],'b--',label='Filtered states')
plt.plot(guard,'k--',label='Guard')
plt.legend()
plt.xlabel(r"Timestep")
plt.ylabel(r"$y$")
plt.title("1D Bouncing Ball Position")
plt.show()

# plt.plot(actual_states[:,1],'k-',label='Actual states')
# plt.plot(measurements[:,1],'r.',label='Measurements')
# plt.plot(filtered_states[:,1],'b--',label='Filtered states')
# plt.legend()
# plt.xlabel(r"Timestep")
# plt.ylabel(r"$\dot{y}$")
# plt.title("1D Bouncing Ball Velocity")
# plt.show()