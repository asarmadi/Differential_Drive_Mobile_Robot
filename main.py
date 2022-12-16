import os
import numpy as np
from robot import Robot

x0 = np.array([0.,0.,0.5])
robot = Robot()

x, u = robot.simulate(x0,5)

if not os.path.isdir('./Figs/'):
   os.mkdir('./Figs/')

robot.animate_motion(x,"./Figs/animation.mp4")
robot.plot(x, u, "./Figs/")
