import casadi as ca
import numpy as np
from models.differentialWheeled import robotDynamics
from controller.controller import Controller

class MPC(Controller):
 def __init__(self, path, robot_dynamics):
     super().__init__()
     self.horizon_len = 100             # MPC horizon that is less than full path horizon
     self.path        = path            # the reference trajectory to be followed
     self.opti        = None            # the optimizer
     self.variables   = {}              # the optimizer variables
     self.costs       = {}            # the MPC cost
     self.tau_b       = 10              # maximum boundry for the wheels torque
     self.dynamics    = robot_dynamics  # the dynamics of the robot defined via casadi
     self.Q           = np.diag([100.0, 100.0, 1.0])    # MPC state cost coefficient matrix
     self.R           = np.diag([0.0, 0.0])    # MPC control cost coefficient matrix

 def setup(self, x0):
     self.opti = ca.Opti()
     self.variables["x"] = self.opti.variable(3, self.horizon_len + 1)
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

         x_diff = self.variables["x"][:, i] - self.path[:, i]
         self.costs["reference_trajectory_tracking"] += ca.mtimes(x_diff.T, ca.mtimes(self.Q, x_diff))


 def get_action(self, x0):
     '''
     This function solves the optimization problem to find the proper action

     Args:
        x0: current state of the robot

     Returns:
        the control action as a 2D array ([tau_r;tau_l])
     '''
     self.setup(x0)
     cost = 0
     for cost_name in self.costs:
         cost += self.costs[cost_name]
     self.opti.minimize(cost)
     option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
     self.opti.solver("ipopt", option)
     opt_sol = self.opti.solve()
     return opt_sol.value(self.variables["u"][:, 0])




