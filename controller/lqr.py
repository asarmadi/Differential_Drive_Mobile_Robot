import numpy as np
from controller.controller import Controller

class LQR(Controller):
 def __init__(self, A, B, Q, R, horizon_len):
  super().__init__()
  self.A = A
  self.B = B
  self.Q = Q
  self.R = R
  self.horizon_len = horizon_len
  self.check_controllability()
  self.K = self.solve_ricatti_equations()

 def solve_ricatti_equations(self):
  '''
  This function solves the backward Riccatti equations for regulator problems of the form
  min xQx + sum(xQx + uRu) subject to xn+1 = Axn + Bun

  Args:
    A, B, Q, R: numpy arrays defining the problem
    horizon_length: length of the horizon

  Returns:
    P: list of numpy arrays containing Pn from N to 0
    K: list of numpy arrays containing Kn from N-1 to 0
  '''
  P = [] #will contain the list of Ps from N to 0
  K = [] #will contain the list of Ks from N-1 to 0

  n=self.horizon_len
  current_p=self.Q
  P.append(current_p)
  while n>0:
    current_K = -1*np.linalg.inv(self.R+np.dot(self.B.T, current_p).dot(self.B)).dot(np.dot(self.B.T,current_p).dot(self.A))
    K.append(current_K)
    current_p = self.Q+np.dot(self.A.T,current_p).dot(self.A)+np.dot(np.dot(np.dot(self.A.T,current_p),self.B),current_K)
    P.append(current_p)
    n=n-1
  return K[::-1]

 def get_action(self, x0, time_idx):
  '''
  This function solves the optimization problem to find the proper action

  Args:
   time_idx: index of time

  Returns:
   the control action as a 2D array
  '''
#  print(self.K)
 # input('enter')
  return self.K[time_idx].dot(x0)

 def check_controllability(self):
    """
    This function check  the controllabilitystate for system
    c=[B AB A^2B]
    """
    c = self.B
    temp = c
    for i in range(self.A.shape[0]):
        c = np.concatenate([c, np.dot(self.A, temp)], axis=1)
        temp = np.dot(self.A,temp)
#    c=np.concatenate([self.B, np.dot(self.A, self.B), np.dot(self.A, self.A).dot(self.B)], axis=1)
    R=np.linalg.matrix_rank(c)
    assert (R==np.linalg.matrix_rank(self.A)), "System is not controllable"
