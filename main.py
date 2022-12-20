import os
import numpy as np
from robot import Robot
from models.differentialWheeled import robotDynamics
from controller.carrot_chase import carrotChase
from controller.mpc import MPC
from controller.path_planner import straightLine

x0 = np.array([0.,0.,0.5,0.,0.])
T  = 5
dt = 0.01
point1 = np.array([0.,1., 0.1, 0., 0.])
point2 = np.array([2.,2., 0.1, 0., 0.])
#cc = carrotChase(point1, point2)
path  = straightLine(point1, point2, int(T/dt)+200)
print(path.shape)
rb    = robotDynamics()
cc    = MPC(path, rb.forward_dynamics_opt(dt))
robot = Robot(cc, dt=dt)

x, u = robot.simulate(x0,T)

if not os.path.isdir('./Figs/'):
   os.mkdir('./Figs/')

robot.animate_motion(x,"./Figs/animation.mp4")
robot.plot(x, u, path, "./Figs/")
