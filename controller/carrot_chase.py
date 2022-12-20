import numpy as np
from controller.controller import Controller

class carrotChase(Controller):
 def __init__(self, w1, w2):
     super().__init__()
     self.delta   = 0.1        # path parameter
     self.w_i     = w1         # way point i
     self.w_i_1   = w2         # way point i+1
     self.k       = 1          # control gain
     self.carrot  = np.array([0.,0.])
 def get_action(self, x):
     '''
     This function generates the proper control actions

     Args:
        x: state of the robot as a 5D array ([x; y; theta; v; omega])

     Returns:
        control action as 2D array ([tau_r;tau_l])
     '''
     theta       = np.arctan2((self.w_i_1[1] - self.w_i[1]),(self.w_i[0]-self.w_i[0]))
     theta_u     = np.arctan2(x[1]-self.w_i[1],x[0]-self.w_i[0])
     R_u         = np.sqrt(np.sum((x[0]-self.w_i[0])**2+(x[1]-self.w_i[1])**2))
     S_1         = np.sqrt(np.sum(self.delta**2+(R_u*np.sin(theta-theta_u))**2))
     zita        = np.arcsin(R_u*np.sin(theta-theta_u)/S_1)
     psi_d       = zita + theta

     r_carrot    = self.delta+R_u*np.cos(theta-theta_u)
     self.carrot = self.w_i + np.array([r_carrot*np.cos(theta),r_carrot*np.sin(theta)])

     psi         = self.k*(psi_d - x[2])
     return np.array([psi,-psi])

 def get_carrot_pose(self, x):
     '''
     This function calculates the pose of the carrot (virtual target point)

     Args:
        x: pose of the robot

     Returns:
        pose of the carrot as 2D array ([x;y])
     '''
     return self.carrot
