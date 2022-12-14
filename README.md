# Differential Drive Mobile Robot
## Modelling
In this section, the dynamic modelling of the robot is described. The matrial is derived from the [Dynamic Modelling of Differential-Drive Mobile Robots using Lagrange and Newton-Euler Methodologies: A Unified Framework](https://www.hilarispublisher.com/open-access/dynamic-modelling-of-differentialdrive-mobile-robots-using-lagrange-and-newtoneuler-methodologies-a-unified-framework-2168-9695.1000107.pdf). The robot configuration is depicted in Following figure:
![Robot_Configuration](/Figs/robot_config.png)

where $\{X_I, Y_I\}$ is the global frame fixed in the environment, $\{X_r,Y_r\}$ the frame attached to the robot, $$q^I=\left[ \matrix{x_a\\y_a\\ \theta} \right] $$
### Kinematic Model


### Dynamic Model
There are three main approaches for modelling the dynamics of a robot: Lagrangian, Newton-Euler, and Kane's method.In this project, we utilize Lagrangian method to find the model.
