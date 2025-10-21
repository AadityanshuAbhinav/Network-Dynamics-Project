# Multi-Robot Collective Navigation Simulation

This repository contains a MATLAB implementation that reproduces the simulations from the paper:

**"Collective Navigation of a Multi-Robot System in an Unknown Environment"** by Erkan Kayacan, Erdal Kayacan, Herman Ramon, and Wouter Saeys (referred to as Olcay et al.)

## Overview

This project implements a distributed control strategy for collective navigation of multiple mobile robots in environments with unknown obstacles. The robots use local sensing and communication to coordinate their movements while avoiding obstacles and maintaining formation cohesion.

### Key Features

- **Flocking-based formation control** using consensus protocols
- **Distributed obstacle avoidance** with local sensing
- **Multiple navigation strategies:**
  - Direct navigation to goal (Status 0)
  - Tangential navigation around obstacles (Status 1)
  - Endpoint turning maneuvers (Status 2)
  - Corner avoidance (Status 3)
- **Three test scenarios:**
  - Zigzag corridor
  - Two circular obstacles
  - Semi-circular obstacle

## Prerequisites

- MATLAB (tested on recent versions, should work on R2018b or later)
- No additional toolboxes required - uses only base MATLAB functions

## Installation

1. Clone this repository:
```bash
git clone https://github.com/AadityanshuAbhinav/Network-Dynamics-Project.git
cd Network-Dynamics-Project
```

2. Open MATLAB and navigate to the project directory

## Usage

### Running a Simulation

Simply run the main script in MATLAB:

```matlab
run_collective_navigation
```

### Selecting Different Scenarios

Edit the `scenario` variable in `run_collective_navigation.m` (line 7):

```matlab
scenario = 'semicircle'; % Options: 'zigzag', 'two_circles', 'semicircle'
```

### Simulation Parameters

Key parameters can be modified in `run_collective_navigation.m`:

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `N` | Number of robots | 12 |
| `dt` | Time step (seconds) | 0.02 |
| `t_max` | Maximum simulation time | 220 |
| `rs` | Sensing range | 15 |
| `rc` | Communication range | 20 |
| `v_max` | Maximum velocity | 4 |
| `d_alpha` | Desired inter-agent distance | 7 |

#### Control Gains

- `c1_alpha`, `c2_alpha`: Formation control gains (gradient and consensus)
- `c1_beta`, `c2_beta`: Obstacle avoidance gains
- `c1_gamma`, `c2_gamma`: Goal tracking gains

## Code Structure

### Main Files

- **`run_collective_navigation.m`**: Main simulation script that initializes agents, sets up scenarios, and runs the simulation loop

### Core Algorithm Functions

- **`calculate_control_inputs.m`**: Computes the total control input for each agent, combining:
  - Formation control (alpha-lattice)
  - Obstacle avoidance (beta-agent)
  - Goal tracking (gamma-agent)

- **`calculate_virtual_goal.m`**: Determines the virtual goal for each agent based on its current status and sensor readings

- **`update_agent_status.m`**: Updates the navigation status of each agent based on obstacle detection:
  - Status 0: Direct navigation
  - Status 1: Tangential navigation
  - Status 2: Endpoint maneuver
  - Status 3: Corner avoidance

### Sensing and Communication

- **`find_neighbors.m`**: Identifies neighboring agents within communication range
- **`detect_obstacles.m`**: Detects obstacle points within sensing range using line segment proximity

### Obstacle Definitions

- **`define_zigzag_obstacle.m`**: Creates a zigzag corridor obstacle
- **`define_circle_obstacle.m`**: Creates circular obstacles
- **`define_semicircle_obstacle.m`**: Creates a semi-circular obstacle

### Mathematical Functions

- **`sigma_norm.m`**: Implements the sigma-norm function (Eq. 4 from the paper)
- **`rho_h.m`**: Implements the bump function (Eq. 5 from the paper)

### Visualization

- **`plot_simulation_state.m`**: Visualizes the current state of the simulation, showing:
  - Agent positions and orientations (colored by status)
  - Communication links between agents
  - Obstacles
  - Goal location

### Utility Functions

- **`evaluate_information.m`**: Evaluates information exchange between agents
- **`gather_information.m`**: Gathers information from neighboring agents
- **`update_backmost_agent.m`**: Updates the status of the rearmost agent

## Algorithm Description

### Control Law

The control input for each agent is composed of three terms:

**u = u_α + u_β + u_γ**

1. **u_α (Formation Control)**: Maintains desired spacing between agents using:
   - Gradient-based term for spacing
   - Consensus-based term for velocity alignment

2. **u_β (Obstacle Avoidance)**: Repulsive force from sensed obstacles

3. **u_γ (Goal Tracking)**: Attractive force toward virtual goal

### Navigation States

Agents switch between different navigation strategies:

- **Status 0 (Direct)**: Move directly toward goal when path is clear
- **Status 1 (Tangential)**: Follow obstacle boundary tangentially
- **Status 2 (Endpoint)**: Execute turning maneuver after passing obstacle
- **Status 3 (Corner)**: Escape from corner/concave regions

## Results

The simulation produces:

1. **Real-time animation** showing agent movement, communication links, and obstacles
2. **Trajectory plot** showing the complete path taken by all agents from start to goal

### Expected Behavior

- Agents start from random positions
- Form a cohesive group while moving toward goal
- Cooperatively navigate around obstacles
- Successfully reach the goal while avoiding collisions

## Parameters Tuning

For different scenarios, you may need to adjust:

- **Control gains**: Balance between formation maintenance, obstacle avoidance, and goal attraction
- **Sensing/Communication ranges**: Affect how agents detect obstacles and coordinate
- **Turning angles** (`delta_turn`): Control sharpness of endpoint maneuvers
- **Stagnation timeout**: Prevents agents from getting stuck

## References

- Olcay Kayacan, E., Kayacan, E., Ramon, H., & Saeys, W. "Collective Navigation of a Multi-Robot System in an Unknown Environment"

## Implementation Notes

- The simulation uses a fixed time-step integration scheme
- Obstacle detection uses closest point on line segment computation
- Virtual goals are computed based on current agent status
- A watchdog timer prevents indefinite stagnation

## License

This is an educational implementation for research purposes.

## Author

Implemented by Aadityanshu Abhinav

## Acknowledgments

This implementation is based on the collective navigation framework described in the paper by Olcay et al.
