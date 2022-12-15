import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Robot:
 def __init__(self):
    self.m_c =  16            # mass of the robot without wheels
    self.I_c =  0.0537        # moment of Inertia of the robot without wheels moment of Inertia
    self.m_w =  0.01          # mass of the wheel
    self.I_w =  0.0011        # moment of inertia of the wheel about the wheel axis
    self.I_m =  0.0023        # moment of inertia of the wheel about the wheel diameter
    self.d   =  0.05          # distance from center of wheels to the CoM
    self.L   =  0.24          # distance from center of wheels to a wheel
    self.R   =  0.095         # wheels radius
    self.robot_height =  0.23 # half of height of the robot
    self.robot_width  =  0.23 # half of width  of the robot
    self.dt  = 0.01           # sampling time
    self.m   = self.m_c+2*self.m_w
    self.I   = self.I_c+self.m_c*(self.d**2)+2*self.m_w*(self.L**2)+2*self.I_m
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
     R = 0.5*np.array([[self.R*np.cos(theta), self.R*np.cos(theta)], 
                       [self.R*np.sin(theta), self.R*np.sin(theta)], 
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
     dtheta = 0.5*(self.R/self.L)*(eta[0] - eta[1])
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
     theta = 0.5*(self.R/self.L)*(x[0] - x[1])
     return x + self.dt*self.forward_dynamics(x, theta, u)

 def simulate(self, x0, T):
     '''
     Simulates the robot for T seconds from initial state x0

     Args:
        x0:  initial state of the robot as a 2D array dot [phi_r; phi_l]
        T:   time horizon
     Returns:
        x and u containing the time evolution of the states and control
     '''
     horizon_length = int(T/self.dt)
     x = np.empty([2, horizon_length])
     u = np.empty([2, horizon_length])
     x[:,0] = x0
     for i in range(horizon_length-1):
         x[:,i+1] = self.step(x[:,i], u[:,i])
     return x, u

 def animate_motion(self, x, save_dir):
     min_dt = 0.1
     if(self.dt < min_dt):
         steps = int(min_dt/self.dt)
         use_dt = int(min_dt * 1000)
     else:
         steps = 1
         use_dt = int(self.dt * 1000)
     plotx = x[:,::steps]

     fig = matplotlib.figure.Figure(figsize=[6,6])
#        matplotlib.backends.backend_agg.FigureCanvasAgg(fig)
     ax = fig.add_subplot(111, autoscale_on=False, xlim=[-5.3,5.3], ylim=[-1.3,1.3])
     ax.grid()

     list_of_lines = []

     line, = ax.plot([], [], 'o', lw=2)
     list_of_lines.append(line)
     line, = ax.plot([], [], 'o', lw=2)
     list_of_lines.append(line)
     line, = ax.plot([], [], 'k-', lw=2)
     list_of_lines.append(line)
     line, = ax.plot([], [], 'k-', lw=2)
     list_of_lines.append(line)
     line, = ax.plot([], [], 'k-', lw=2)
     list_of_lines.append(line)
     line, = ax.plot([], [], 'k-', lw=2)
     list_of_lines.append(line)

     def animate(i):
            for l in list_of_lines: #reset all lines
                l.set_data([],[])

            theta  = 0.5*(self.R/self.L)*(plotx[0,i] - plotx[1,i])
            q      = self.forward_kinematic(theta, plotx[:,i])
            x_a    = q[0]
            y_a    = q[1]

            x_wr   = x_a+self.L*np.sin(q[3])
            y_wr   = y_a+self.L*np.cos(q[3])

            x_wl   = x_a-self.L*np.sin(q[3])
            y_wl   = y_a+self.L*np.cos(q[3])

            x_sr   = x_a+self.robot_width*np.sin(q[3])
            y_sr   = y_a+self.robot_width*np.cos(q[3])

            x_sl   = x_a-self.robot_width*np.sin(q[3])
            y_sl   = y_a+self.robot_width*np.cos(q[3])
            x_p1   = x_sr+self.robot_height*np.cos(q[3])
            y_p1   = y_sr+self.robot_height*np.sin(q[3])
            x_p2   = x_sr-self.robot_height*np.cos(q[3])
            y_p2   = y_sr-self.robot_height*np.sin(q[3])
            x_p3   = x_sl+self.robot_height*np.cos(q[3])
            y_p3   = y_sl+self.robot_height*np.sin(q[3])
            x_p4   = x_sl-self.robot_height*np.cos(q[3])
            y_p4   = y_sl-self.robot_height*np.sin(q[3])

            list_of_lines[0].set_data([x_wr, x_wr], [y_wr, y_wr])
            list_of_lines[1].set_data([x_wl, x_wl], [y_wl, y_wl])
            list_of_lines[2].set_data([x_p1,x_p2],[y_p1, y_p2])
            list_of_lines[3].set_data([x_p1,x_p3],[y_p1, y_p3])
            list_of_lines[4].set_data([x_p3,x_p4],[y_p3, y_p4])
            list_of_lines[5].set_data([x_p4,x_p1],[y_p4, y_p1])

            return list_of_lines

     def init():
         return animate(0)

     ani = animation.FuncAnimation(fig, animate, np.arange(0, len(plotx[0,:])),
         interval=use_dt, blit=True, init_func=init)

     f = save_dir
     writervideo = animation.FFMpegWriter(fps=60)
     ani.save(f, writer=writervideo)

     plt.close(fig)
