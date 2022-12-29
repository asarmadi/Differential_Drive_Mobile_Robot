import numpy as np
from controller.controller import Controller

class carrotChase(Controller):
 def __init__(self, w1, w2):
     super().__init__()
     self.delta   = 1                    # path parameter
     self.w_i     = w1                     # way point i
     self.w_i_1   = w2                     # way point i+1
     self.k1      = 10                      # angle error control gain
     self.k2      = 0                      # distance error control gain
     self.carrot  = np.array([0.,0.])      # position of the carrot
     self.action  = np.array([0.01,0.01])  # cumulative action signal

 def get_action(self, x):
     '''
     This function generates the proper control actions

     Args:
        x: state of the robot as a 5D array ([x; y; theta; v; omega])

     Returns:
        control action as 2D array ([tau_r;tau_l])
     '''
     theta       =  np.arctan2((self.w_i_1[1] - self.w_i[1]),(self.w_i_1[0]-self.w_i[0]))
     theta_u     =  np.arctan2(x[1]-self.w_i[1],x[0]-self.w_i[0])
     beta        =  theta - theta_u
     R_u         =  np.sqrt((x[0]-self.w_i[0])**2+(x[1]-self.w_i[1])**2)

#     R           =  R_u*np.cos(beta)

     R           =  np.sqrt(R_u**2 - (R_u*np.sin(beta))**2 )
     S           =  np.array([(R+self.delta)*np.cos(theta),(R+self.delta)*np.sin(theta)])
     psi_d       =  np.arctan2(S[1] + x[1],S[0] + x[0])

     r_carrot    =  self.delta+R_u*np.cos(theta-theta_u)
     self.carrot =  self.w_i[0:2] + np.array([r_carrot*np.cos(theta),r_carrot*np.sin(theta)])

     e           =  R_u*np.sin(beta)

     d_w_i_1     =  np.sqrt((x[0]-self.w_i_1[0])**2+(x[1]-self.w_i_1[1])**2)
     u           =  self.k1 * (psi_d - x[2]) - self.k2 * e
     if u > 0:
      self.action =  np.array([0.1+u,0.1])
     else:
      self.action =  np.array([0.1,0.1-u])

     print(d_w_i_1)
#     input('enter')
     if d_w_i_1 < 6.:
      self.action = np.array([0.,0.])

     return self.action

 def get_carrot(self):
     '''
     This function calculates the pose of the carrot (virtual target point)

     Args:
        x: pose of the robot

     Returns:
        pose of the carrot as 2D array ([x;y])
     '''
     return self.carrot
