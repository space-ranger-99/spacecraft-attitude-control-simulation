# Attitude Simulation for Spacecraft Control

This repository contains a MATLAB script attitude_simulation.m that simulates the attitude control of a spacecraft. The script demonstrates the use of a control law for the attitude control of a spacecraft, defines the system parameters, solves the attitude dynamics, and plots the results for both the true and desired attitudes, as well as the attitude error over time.

### Main Functions

The script consists of the following main functions:

- `init_params`: Initializees the system parameters.
- `solve_dynamics`: Solves the attitude dynamics using the ode45 solver.
- `attitude_dynamics`: Calculates the attitude dynamics based on the control law and system parameters.
- `reference_dynamics`: Defines the desired reference dynamics for the spacecraft's attitude.
- `tilde_dynamics`: Calculates the tilde dynamics for error computation.
- `control_law`: Implements the control law.
- `plot_and_save_results`: Plots and saves the simulation results.

### Usage

To use the script, simply call the function `attitude_simulation()` in MATLAB. The script will run the simulation, plot the results, and save a figure with the simulation results as a PNG file named `attitude_simulation_results.png`.

```matlab
attitude_simulation()
```

### Dependencies

The script requires MATLAB and the ODE45 solver, which is included in the core MATLAB package.

### Author

**Alexander Little**

- 🏫 Toronto Metropolitan University
- 📧 corbyn.little@torontomu.ca
- 💼 [LinkedIn](https://www.linkedin.com/in/aclittle/)
- 📚 [GitHub](https://github.com/space-ranger-99)

*Version 1.0*
