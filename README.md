# Differential Drive Mobile Robot
## Modelling
In this section, the dynamic modelling of the robot is described. The matrial is derived from the [Dynamic Modelling of Differential-Drive Mobile Robots using Lagrange and Newton-Euler Methodologies: A Unified Framework](https://www.hilarispublisher.com/open-access/dynamic-modelling-of-differentialdrive-mobile-robots-using-lagrange-and-newtoneuler-methodologies-a-unified-framework-2168-9695.1000107.pdf). The robot configuration is depicted in Following figure:
![Robot_Configuration](/Figs/robot_config.png)

where $\{X_I, Y_I\}$ is the global frame fixed in the environment (inertial frame), $\{X_r,Y_r\}$ the frame attached to the robot (robot frame), $q^I=\left[ \matrix{x_a \cr y_a \cr \theta} \right]$ robot pose in the inertial frame. The pose of any point in the robot frame ($X^r = \left[ \matrix{x^r \cr y^r \cr {\theta}^r} \right]$) and the inertial frame ($X^I = \left[ \matrix{x^I \cr y^I \cr \theta^I} \right]$) are related as
$$ X^I = R(\theta) X^r = \left[ \matrix{cos\theta & -sin\theta & 0 \cr sin\theta & cos\theta & 0 \cr 0 & 0 & 1} \right] $$
$$\dot{X^I} = R(\theta) \dot{X^r}$$
### Kinematic Model
The motion of the robot is characterized by the following two non-holonomic constraints:

1. No lateral slip: The robot only moves forward and backward not sideward:
$$\dot{y_a^r}=0 \rightarrow -\dot x_a sin\theta + \dot y_a cos\theta=0$$
2. Pure rolling constraint: Each wheel has one contact to the ground (no slipping or skidding)

### Dynamic Model
There are three main approaches for modelling the dynamics of a robot: Lagrangian, Newton-Euler, and Kane's method.In this project, we utilize Lagrangian method to find the model.
