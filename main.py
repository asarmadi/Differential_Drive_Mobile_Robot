import os
import argparse
import numpy as np
from models.ddmr import DDMR
from models.simple_ddmr import SDDMR
from models.quad_2d import Quad2D
from controller.carrot_chase import carrotChase
from controller.mpc import MPC
from controller.path_planner import straightLine

parser = argparse.ArgumentParser(description='Differential Wheeled Robot Control')
parser.add_argument('--T',      type=float, default = 5.0,   help='Total time horizon')
parser.add_argument('--dt',     type=float, default = 0.01,  help='Sampling time')
parser.add_argument('--robot', type=str,   default = "mpc", help='Robot Type')
parser.add_argument('--method', type=str,   default = "mpc", help='Control method')
args = parser.parse_args()

if args.robot == 'ddmr':
   point1 = np.array([-1., 1., 0.1, 0., 0.])
   point2 = np.array([60. , 60., 0.1, 0., 0.])
   x0 = np.array([0.,0.,0.5,0.,0.])
   Q  = np.diag([10.0, 10.0, 1.0, 0.0, 0.0])
   robot = DDMR(dt=args.dt)
elif args.robot == 'quad2d':
   point1 = np.array([-1., 1., 0.1, 0., 0., 0.])
   point2 = np.array([60. , 60., 0.1, 0., 0., 0.])
   x0 = np.array([0.,0.,0.5,0.,0.,0.])
   Q  = np.diag([10.0, 10.0, 1.0, 0.0, 0.0, 0.0])
   robot = Quad2D(dt=args.dt)
elif args.robot == 'sddmr':
   point1 = np.array([-1., 1., 0.1, 0., 0.])
   point2 = np.array([60. , 60., 0.1, 0., 0.])
   x0 = np.array([0.,0.,2.1,0.,0.])
   Q  = np.diag([10.0, 10.0, 1.0, 0.0, 0.0])
   robot = SDDMR(dt=args.dt)

path  = straightLine(point1, point2, int(args.T/args.dt)+20)

if args.method == 'carrot_chasing':
 cc = carrotChase(point1, point2)
elif args.method == 'mpc':
 cc    = MPC(path, robot.forward_dynamics_opt(args.dt), Q)

robot.set_controller(cc)

x, u, c = robot.simulate(x0,args.T)

if not os.path.isdir('./Figs/'):
 os.mkdir('./Figs/')

robot.animate_motion(x,u,"./Figs/animation.mp4")
robot.plot(x, u, c, path, "./Figs/")
