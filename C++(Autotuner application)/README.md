# PSO Single Leg Optimization

This application is designed to test PSO optimization on a single leg. It is run on a host system, which sends commands to a Raspberry Pi. The Raspberry Pi then forwards these commands to the motors uisng a Pi3hat. The gains are tuned based on genetic algorithm PSO

## Prerequisites

Before running this application, make sure you have the following:

- Raspberry Pi with the necessary setup
- Motor control hardware connected to the Raspberry Pi using moteus
- Ethernet communication enabled on the Raspberry Pi
- A kill switch for safety

## Installation

1. Clone this repository to your local machine.
2. Make sure you have all the required dependencies installed.
3. Compile and build the application using the provided Makefile. (I have removed the secific information)

## Usage

To use this application, follow these steps:

1. Set the IP address and port number for the peer connection in the `PEER_IP` and `PEER_PORT` constants, respectively.
2. Set the port number for the self connection in the `SELF_PORT` constant.
3. Adjust the control time period and buffer size if needed by modifying the `PERIOD_MS` and `BUFFER_SIZE` constants.
4. Run the application on the host system.

## Autotuner Parameters

The application uses the following autotuner parameters:

- `MAX_GENERATIONS`: Maximum number of generations for the PSO algorithm.
- `MAX_AGENTS`: Maximum number of agents for the PSO algorithm.
- `c1_swarm_`, `c2_swarm_`: Constants for the PSO algorithm.
- `gamma_1_swarm_`, `gamma_2_swarm_`: Gamma values for the PSO algorithm.
- `omega_swarm_`, `omega_swarm_max_`, `omega_swarm_min_`: Omega values for the PSO algorithm.
- `max_velocity_gain_`: Maximum velocity gain for the PSO algorithm.

## Autotuner FSM

The application uses a finite state machine (FSM) for the autotuner process. The FSM has the following states:

- State 0: Reset State FSM
- State 1: PSO - I step
- State 2: Trajectory State FSM
- State 3: PSO - II step

For each state, specific actions and transitions are defined.

## Elliptical Trajectory Generator

The application uses an elliptical trajectory generator for testing. It calculates the position and velocity of the leg based on a given theta value.

## Author

This application is developed by Prakhar Goel.

