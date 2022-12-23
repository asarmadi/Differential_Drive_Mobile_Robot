# Nonlinear Robot Control

## Robots
Two robots are considered: Differential Wheeled Robot, and 2D quadrotor.

## Modelling
A detailed summary of the modelling of the robot is reported in the [report file](https://github.com/asarmadi/Differential_Drive_Mobile_Robot/blob/main/Docs/report.pdf). The matrial of the report are mostly taken from [Dynamic Modelling of Differential-Drive Mobile Robots using Lagrange and Newton-Euler Methodologies: A Unified Framework](https://www.hilarispublisher.com/open-access/dynamic-modelling-of-differentialdrive-mobile-robots-using-lagrange-and-newtoneuler-methodologies-a-unified-framework-2168-9695.1000107.pdf)

## Simulation
The parameters for the simulation are derived form the [Dynamics modeling and sliding mode control of tractor-trailer wheeled mobile robots subject to wheels slip](https://www.sciencedirect.com/science/article/pii/S0094114X18319062)

## Control
The follwing controllers are implemented:

[-] Carrot-chasing: A geometry-based controller

[-] MPC: An optimization-based controller 

[-] Backstepping:

## Runing the code
To run any of the algorithms, you can run the following command:

``python main.py --T 5 --dt 0.01 --method mpc -- robot ddmr``

- `--T`       is an integer showing the duration of the motion in seconds
- `--dt`      is the sampling time is seconds
- `--method`  can be either 'mpc' or 'carrot_chasing'
- `--robot`   can be either 'ddmr' or 'quad2d'
