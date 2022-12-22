import casadi as ca
import numpy as np
from controller.controller import Controller

class MPC(Controller):
 def __init__(self, global_path, robot_dynamics):
     super().__init__()
     self.horizon_len = 10                          # MPC horizon that is less than full path horizon
     self.global_path = global_path                  # the reference trajectory to be followed
     self.opti        = None                         # the optimizer
     self.variables   = {}                           # the optimizer variables
     self.costs       = {}                           # the MPC cost
     self.tau_b       = 10                                        # maximum boundry for the wheels torque
     self.dynamics    = robot_dynamics                           # the dynamics of the robot defined via casadi
     self.Q           = np.diag([10.0, 10.0, 1.0, 0.0, 0.0]) # MPC state cost coefficient matrix
     self.R           = np.diag([0.0, 0.0])                      # MPC control cost coefficient matrix

 def setup(self, x0, ref_path):
  '''
  This function sets up the optimization problem via casadi library

  Args:
   x0:       current pose of the robot. It's an array of size 5,
   ref_path: reference path as an array of size 5*horizon_len

  Returns:
   None
  '''
  self.opti = ca.Opti()
  self.variables["x"] = self.opti.variable(len(x0),  self.horizon_len + 1)
  self.variables["u"] = self.opti.variable(2, self.horizon_len)
  self.opti.subject_to(self.variables["x"][:,0] == x0)
  self.costs["reference_trajectory_tracking"] = 0
  self.costs["input_stage"] = 0

  for i in range(self.horizon_len):
      self.opti.subject_to(self.variables["u"][0, i] <= self.tau_b)
      self.opti.subject_to(-self.tau_b <= self.variables["u"][0, i])
      self.opti.subject_to(self.variables["u"][1, i] <= self.tau_b)
      self.opti.subject_to(-self.tau_b <= self.variables["u"][1, i])

      self.opti.subject_to(self.variables["x"][:,i+1] == self.dynamics(self.variables["x"][:,i],self.variables["u"][:,i]))
      self.costs["input_stage"] += ca.mtimes(
          self.variables["u"][:, i].T, ca.mtimes(self.R, self.variables["u"][:, i])
      )

      x_diff = self.variables["x"][:, i] - ref_path[:, i]
      self.costs["reference_trajectory_tracking"] += ca.mtimes(x_diff.T, ca.mtimes(self.Q, x_diff))


 def get_action(self, x0):
  '''
  This function solves the optimization problem to find the proper action

  Args:
   x0: current state of the robot

  Returns:
   the control action as a 2D array ([tau_r;tau_l])
  '''
  local_path = self.gen_local_path(x0)
  if local_path == []:
     return np.array([0., 0.])

  self.setup(x0, local_path)
  cost = 0
  for cost_name in self.costs:
      cost += self.costs[cost_name]
  self.opti.minimize(cost)
  option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
  self.opti.solver("ipopt", option)
  opt_sol = self.opti.solve()
  return opt_sol.value(self.variables["u"][:, 0])

 def gen_local_path(self, x):
  '''
  This function generates a local path given the current pose of robot (i.e., x0)
  for MPC horizon

  Args:
   x: current pose of the robot

  Returns:
   An array of size 5*horizon_len
  '''
  XT = np.tile(x,(self.global_path.shape[1],1)).T
  diff = np.linalg.norm(self.global_path - XT,axis=0)
  idx  = np.argmin(diff)
  if idx > (self.global_path.shape[1] - self.horizon_len): # we check whether the robot reached the goal
     return []
  return self.global_path[:,idx:idx+self.horizon_len+1]

