import numpy as np

class Robot:
 def __init__(self):
    self.m_c =  # mass of the robot without wheels
    self.I_c =  # moment of Inertia of the robot without wheels moment of Inertia
    self.m_w =  # mass of the wheel
    self.I_w =  # moment of inertia of the wheel about the wheel axis
    self.I_m =  # moment of inertia of the wheel about the wheel diameter
    self.d   =  # distance from center of wheels to the CoM
    self.L   =  # distance from center of wheels to a wheel
    self.R   =  # wheels radius
    self.dt  = 0.01 # sampling time
    self.m   = m_c+2*m_w
    self.I   = I_c+m_c*(d**2)+2*m_w*(L**2)+2*I_m
    self.M   = np.array([[self.I_w+((self.R**2)*(self.m*self.L**2+self.I))/(4*self.L**2),((self.R**2)*(self.m*self.L**2-self.I))/(4*self.L**2)],
                         [((self.R**2)*(self.m*self.L**2-self.I))/(4*self.L**2),self.I_w+((self.R**2)*(self.m*self.L**2+self.I))/(4*self.L**2)] ])
    self.B   = np.array([[1,0], [0,1]])

 def forward_kinematic(self, theta, d_phi):
     '''
     Calculates the forward kinematics of the robot

     Args:
         theta: The orientation of the robot
         d_phi: The vector of wheels angular velocities (dot [phi_r; phi_l])

     Returns:
         Pose vector as dot [x_a; y_a; theta; phi_r; phi_l]
     '''
     R = 0.5*np.array([[self.R*cos(theta), self.R*cos(theta)], 
                       [self.R*sin(theta), self.R*sin(theta)], 
                       [self.R/self.L, -self.R/self.L], 
                       [2, 0],
                       [0, 2] ])
     return R.dot(d_phi)

 def forward_dynamics(self, eta, theta, tau):
     '''
     Calculates the forward kinematics of the robot

     Args:
         theta: The orientation of the robot
         d_phi: The vector of wheels angular velocities (dot [phi_r; phi_l])

     Returns:
         Pose vector as dot [x_a; y_a; theta; phi_r; phi_l]
     '''
     V = np.array([[0, (self.R**2*self.m_c*self.d*dtheta)/(2*self.L)], 
                  [-(self.R**2*self.m_c*self.d*dtheta)/(2*self.L),0] ])
     return np.linalg.inv(self.M)@(self.B@tau - V@eta)

 def step(self, ):
