import numpy as np
from tqdm import tqdm

class Robot:
 def __init__(self, dt):
    self.dt           =  dt            # sampling time

 def forward_dynamics(self, x, u):
     '''
     Calculates the forward dynamics of the robot

     Args:
         x:   The vector of the robot linear and angular velocities ([x; y; theta; v; omega])
         u:   The vector of motor torques ([tau_r; tau_l])

     Returns:
         A 5D array of the updated state
     '''
     pass

 def step(self, x, u):
     '''
     Integrates the robot for one step of self.dt

     Args:
        x:   state of the robot as a 5D array [x; y; theta; v; omega]
        u:   control input as a 2D array [tau_r; tau_l]
     Returns:
        The state of the robot after integration
     '''
     return x + self.dt*self.forward_dynamics(x, u)

 def simulate(self, x0, T):
     '''
     Simulates the robot for T seconds from initial state x0

     Args:
        x0:  initial state of the robot as a 5D array ([x; y; theta; v; omega])
        T:   time horizon
     Returns:
        x and u containing the time evolution of the states and control
     '''
     horizon_length = int(T/self.dt)
     x = np.zeros([self.n_dim, horizon_length])
     u = np.zeros([2, horizon_length])
     c = np.zeros([2, horizon_length])
     x[:,0] = x0
     for i in tqdm(range(horizon_length-1)):
         u[:, i]    = self.controller.get_action(x[:,i])
#         c[:, i]    = self.controller.get_carrot()
         x[:,i+1]   = self.step(x[:,i], u[:,i])
     return x, u, c

 def set_controller(self, cc):
     '''
     This function sets the controller to the input

     Args:
       cc: controller object

     Returns:
       None
     '''
     self.controller = cc
