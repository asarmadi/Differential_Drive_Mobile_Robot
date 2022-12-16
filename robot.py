import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Robot:
 def __init__(self):
    self.m_c =  16            # mass of the robot without wheels
    self.I_c =  0.0537        # moment of Inertia of the robot without wheels moment of Inertia
    self.m_w =  0.25          # mass of the wheel
    self.I_w =  0.0011        # moment of inertia of the wheel about the wheel axis
    self.I_m =  0.0023        # moment of inertia of the wheel about the wheel diameter
    self.d   =  0.05          # distance from center of wheels to the CoM
    self.L   =  0.24          # distance from center of wheels to a wheel
    self.R   =  0.095         # wheels radius
    self.robot_height =  0.23 # half of height of the robot
    self.robot_width  =  0.23 # half of width  of the robot
    self.dt  = 0.01           # sampling time
    self.n_dim= 3             # number of state dimensions
    self.m   = self.m_c+2*self.m_w
    self.I   = self.I_c+self.m_c*(self.d**2)+2*self.m_w*(self.L**2)+2*self.I_m
    self.M   = np.array([[self.I_w+((self.R**2)*(self.m*self.L**2+self.I))/(4*self.L**2),((self.R**2)*(self.m*self.L**2-self.I))/(4*self.L**2)],
                         [((self.R**2)*(self.m*self.L**2-self.I))/(4*self.L**2),self.I_w+((self.R**2)*(self.m*self.L**2+self.I))/(4*self.L**2)] ])
    self.B   = np.array([[1,0], [0,1]])

 def forward_kinematic(self, x):
     '''
     Calculates the forward kinematics of the robot

     Args:
         x: The 3d state vector ([v; omega; theta])

     Returns:
         Pose vector as dot [x; y; theta]
     '''
     R = np.array([ [np.cos(x[2]),0,0], [np.sin(x[2]),0,0], [0,1,0]])
     return R@x

 def forward_dynamics(self, x, u):
     '''
     Calculates the forward dynamics of the robot

     Args:
         x:   The vector of the robot linear and angular velocities ([v; omega; theta])
         u:   The vector of motor torques ([tau_r; tau_l])

     Returns:
         Derivative of eta as ddot[phi_r; phi_l]
     '''
     q    = np.zeros([3,])
     q[0] = (self.R**2/(self.m*self.R**2+2*self.I_w))*(self.m_c*self.d*x[1]**2+(1/self.R)*(u[0]+u[1]))
     q[1] = (self.R**2/(self.I*self.R**2+2*self.I_w*self.L**2))*(-self.m_c*self.d*x[1]*x[0]+(self.L/self.R)*(u[0]-u[1]))
     q[2] = q[1]
     return q

 def step(self, x, u):
     '''
     Integrates the robot for one step of self.dt

     Args:
        x:   state of the robot as a 2D array dot [phi_r; phi_l]
        u:   control input as a 2D array [tau_r; tau_l]
     Returns:
        The state of the robot after integration
     '''
     return x + self.dt*self.forward_dynamics(x, u)

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
     x = np.zeros([self.n_dim, horizon_length])
     u = np.ones([self.n_dim, horizon_length])*0.1
     x[:,0] = x0
     for i in range(horizon_length-1):
         x[:,i+1] = self.step(x[:,i], u[:,i])
     return x, u


 def plot(self, x, u, save_dir):
     '''
     This function plots the robot state and action 

     Args:
        x:   state of the robot as a 2D array dot [phi_r; phi_l]
        save_dir:   the directory to be used for saving the animation
     Returns:
        None, it saves the generated plots in save_dir directory
     '''
     q = np.empty([self.n_dim, x.shape[1]])
     for i in range(x.shape[1]):
         q[:,i] =  self.forward_kinematic(x[:,i])
     plt.figure(0)
     plt.plot(q[0,:],q[1,:])
     plt.xlabel('X')
     plt.ylabel('Y')
     plt.savefig(save_dir+'x_y.png')

     plt.figure(1)
     plt.plot(q[2,:])
     plt.ylabel('Phi')
     plt.savefig(save_dir+'phi.png')

 def animate_motion(self, x, save_dir):
     '''
     Generates an animation of the robot states

     Args:
        x:  initial state of the robot as a 2D array dot [phi_r; phi_l]
        save_dir:   the directory to be used for saving the animation
     Returns:
        None -> It saves the generated motion in the save_dir directory
     '''

     min_dt = 0.01
     if(self.dt < min_dt):
         steps = int(min_dt/self.dt)
         use_dt = int(min_dt * 1000)
     else:
         steps = 1
         use_dt = int(self.dt * 1000)
     plotx = x[:,::steps]

     fig = matplotlib.figure.Figure(figsize=[6,6])
     ax = fig.add_subplot(111, autoscale_on=False, xlim=[-1.3,1.3], ylim=[-1.3,1.3])
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

            q      = self.forward_kinematic(plotx[:,i])
            x_a    = q[0]
            y_a    = q[1]
            theta  = plotx[2,i]

            x_wr   = x_a+self.L*np.sin(theta)
            y_wr   = y_a-self.L*np.cos(theta)

            x_wl   = x_a-self.L*np.sin(theta)
            y_wl   = y_a+self.L*np.cos(theta)

            x_sr   = x_a+self.robot_width*np.sin(theta)
            y_sr   = y_a-self.robot_width*np.cos(theta)

            x_sl   = x_a-self.robot_width*np.sin(theta)
            y_sl   = y_a+self.robot_width*np.cos(theta)

            x_p1   = x_sr+self.robot_height*np.cos(theta)
            y_p1   = y_sr+self.robot_height*np.sin(theta)

            x_p2   = x_sr-self.robot_height*np.cos(theta)
            y_p2   = y_sr-self.robot_height*np.sin(theta)

            x_p3   = x_sl+self.robot_height*np.cos(theta)
            y_p3   = y_sl+self.robot_height*np.sin(theta)

            x_p4   = x_sl-self.robot_height*np.cos(theta)
            y_p4   = y_sl-self.robot_height*np.sin(theta)

            list_of_lines[0].set_data([x_wr, x_wr], [y_wr, y_wr])
            list_of_lines[1].set_data([x_wl, x_wl], [y_wl, y_wl])
            list_of_lines[2].set_data([x_p1,x_p2],[y_p1, y_p2])
            list_of_lines[3].set_data([x_p1,x_p3],[y_p1, y_p3])
            list_of_lines[4].set_data([x_p3,x_p4],[y_p3, y_p4])
            list_of_lines[5].set_data([x_p4,x_p2],[y_p4, y_p2])

            return list_of_lines

     def init():
         return animate(0)

     ani = animation.FuncAnimation(fig, animate, np.arange(0, len(plotx[0,:])),
         interval=use_dt, blit=True, init_func=init)

     f = save_dir
     writervideo = animation.FFMpegWriter(fps=60)
     ani.save(f, writer=writervideo)

     plt.close(fig)
