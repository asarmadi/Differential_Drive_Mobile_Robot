import os
import numpy as np
from robot import Robot
from controller.carrot_chase import carrotChase

x0 = np.array([0.,0.,0.5])
point1 = np.array([-1.,1.])
point2 = np.array([2.,1.])
cc = carrotChase(point1, point2)
robot = Robot(cc)

x, u = robot.simulate(x0,5)

if not os.path.isdir('./Figs/'):
   os.mkdir('./Figs/')

robot.animate_motion(x,"./Figs/animation.mp4")
robot.plot(x, u, "./Figs/")
