import casadi as ca
import numpy as np

class robotDynamics:
 def __init__(self):
  self.m_c          =  16            # mass of the robot without wheels
  self.I_c          =  0.0537        # moment of Inertia of the robot without wheels moment of Inertia
  self.m_w          =  0.25          # mass of the wheel
  self.I_w          =  0.0011        # moment of inertia of the wheel about the wheel axis
  self.I_m          =  0.0023        # moment of inertia of the wheel about the wheel diameter
  self.d            =  0.05          # distance from center of wheels to the CoM
  self.L            =  0.24          # distance from center of wheels to a wheel
  self.R            =  0.095         # wheels radius
  self.robot_height =  0.23          # half of height of the robot
  self.robot_width  =  0.23          # half of width  of the robot
  self.dt           =  0.01           # sampling time
  self.n_dim        =  3              # number of state dimensions
  self.m            =  self.m_c+2*self.m_w
  self.I            =  self.I_c+self.m_c*(self.d**2)+2*self.m_w*(self.L**2)+2*self.I_m
  self.M            =  np.array([[self.I_w+((self.R**2)*(self.m*self.L**2+self.I))/(4*self.L**2),((self.R**2)*(self.m*self.L**2-self.I))/(4*self.L**2)],
                                [((self.R**2)*(self.m*self.L**2-self.I))/(4*self.L**2),self.I_w+((self.R**2)*(self.m*self.L**2+self.I))/(4*self.L**2)] ])
# @staticmethod
 def forward_dynamics(self, x, u, timestep):
  x_next = np.ndarray(shape=(3,), dtype=float)
  x_next[0] = x[0] + ((self.R**2/(self.m*self.R**2+2*self.I_w))*(self.m_c*self.d*x[1]**2+(1/self.R)*(u[0]+u[1]))) * timestep
  x_next[1] = x[1] + ((self.R**2/(self.I*self.R**2+2*self.I_w*self.L**2))*(-self.m_c*self.d*x[1]*x[0]+(self.L/self.R)*(u[0]-u[1]))) * timestep
  x_next[2] = x[2] + x[1] * timestep
  return x_next

 #@staticmethod
 def forward_dynamics_opt(self, timestep):
  x_symbol = ca.SX.sym("x", 3)
  u_symbol = ca.SX.sym("u", 2)
  x_symbol_next = x_symbol[0] + ((self.R**2/(self.m*self.R**2+2*self.I_w))*(self.m_c*self.d*x_symbol[1]**2+(1/self.R)*(u_symbol[0]+u_symbol[1]))) * timestep
  y_symbol_next = x_symbol[1] + ((self.R**2/(self.I*self.R**2+2*self.I_w*self.L**2))*(-self.m_c*self.d*x_symbol[1]*x_symbol[0]+(self.L/self.R)*(u_symbol[0]-u_symbol[1]))) * timestep
  theta_symbol_next = x_symbol[2] + x_symbol[1] * timestep
  state_symbol_next = ca.vertcat(x_symbol_next, y_symbol_next, theta_symbol_next)
  return ca.Function("differential_wheeled_dynamics", [x_symbol, u_symbol], [state_symbol_next])

