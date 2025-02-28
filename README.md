# FallingLeaf
Analysis and Control of Nonlinear Flight Systems Project: Flight control laws’ susceptibility to the falling leaf motion.

**Following work has been performed:**
- Implementation of a baseline controller from Chakraborty 2010
- Trim point identification
- Linearization of the nonlinear model aroudn the trim points
- Development of a LQR controller
- Design of a Monte Carlo Simulation to validate the applicability of the LQR to the nonlinear dynamcis
- Development of a nonlinear MPC
- Analysis of Dissipativity for the terminal region of the MPC

**Implementation Details:**
- sim folder: Contains all .slx file, the physical model and all (linear and nonlinear) controller laws
- lib folder: Contains library functions, that were used throughout the project
- data folder: Contains trim point data and results from the Monte Carlo simulation.
