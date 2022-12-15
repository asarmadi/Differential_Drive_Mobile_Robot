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

 def forward_kinematic(self, theta, eta):
     '''
     Calculates the forward kinematics of the robot

     Args:
         theta: The orientation of the robot
         eta: The vector of wheels angular velocities (dot [phi_r; phi_l])

     Returns:
         Pose vector as dot [x_a; y_a; theta; phi_r; phi_l]
     '''
     R = 0.5*np.array([[self.R*cos(theta), self.R*cos(theta)], 
                       [self.R*sin(theta), self.R*sin(theta)], 
                       [self.R/self.L, -self.R/self.L], 
                       [2, 0],
                       [0, 2] ])
     return R.dot(eta)

 def forward_dynamics(self, eta, theta, tau):
     '''
     Calculates the forward dynamics of the robot

     Args:
         eta:   The vector of wheels angular velocities (dot [phi_r; phi_l])
         theta: The orientation of the robot
         tau:   The vector of motor torques ([tau_r; tau_l])

     Returns:
         Derivative of eta as ddot[phi_r; phi_l]
     '''
     V = np.array([[0, (self.R**2*self.m_c*self.d*dtheta)/(2*self.L)], 
                  [-(self.R**2*self.m_c*self.d*dtheta)/(2*self.L),0] ])
     return np.linalg.inv(self.M)@(self.B@tau - V@eta)

 def step(self, x, u):
     '''
     Integrates the robot for one step of self.dt

     Args:
        x:   state of the robot as a 2D array dot [phi_r; phi_l]
        u:   control input as a 2D array [tau_r; tau_l]
     Returns:
        The state of the robot after integration
     '''
     theta = 0.5*(self.R/self.L)(x[0] - x[1])
     return x + self.dt*self.forward_dynamics(x, theta, u)

def simulate(self, x0, T):
     '''
     Simulates the robot for T seconds from initial state x0

     Args:
        x0:  initial state of the robot as a 2D array dot [phi_r; phi_l]
        T:   time horizon
     Returns:
        The state of the robot after integration
     '''



