import matplotlib
import numpy as np
import casadi as ca
from tqdm import tqdm
from models.robot import Robot
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class DDMR(Robot):
 def __init__(self, dt):
    super().__init__(dt)
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
    self.n_dim        =  5             # number of state dimensions
    self.m            =  self.m_c+2*self.m_w
    self.I            =  self.I_c+self.m_c*(self.d**2)+2*self.m_w*(self.L**2)+2*self.I_m
    self.M            =  np.array([[self.I_w+((self.R**2)*(self.m*self.L**2+self.I))/(4*self.L**2),((self.R**2)*(self.m*self.L**2-self.I))/(4*self.L**2)],
                                  [((self.R**2)*(self.m*self.L**2-self.I))/(4*self.L**2),self.I_w+((self.R**2)*(self.m*self.L**2+self.I))/(4*self.L**2)] ])
    self.B            =  np.array([[1,0], [0,1]])

 def forward_dynamics_opt(self, timestep):
  '''
  This function implements the forward dynamics using Casadi

  Args:
    timestep: the timestep to be used for the calculations

  Returns:
    A Casadi function
  '''
  x_symbol = ca.SX.sym("x", self.n_dim)
  u_symbol = ca.SX.sym("u", 2)

  x_symbol_next     = x_symbol[0] + x_symbol[3]*np.cos(x_symbol[2]) * timestep
  y_symbol_next     = x_symbol[1] + x_symbol[3]*np.sin(x_symbol[2]) * timestep
  theta_symbol_next = x_symbol[2] + x_symbol[4] * timestep
  v_symbol_next     = x_symbol[3] + ((self.R**2/(self.m*self.R**2+2*self.I_w))*(self.m_c*self.d*x_symbol[4]**2+(1/self.R)*(u_symbol[0]+u_symbol[1]))) * timestep
  omega_symbol_next = x_symbol[4] + ((self.R**2/(self.I*self.R**2+2*self.I_w*self.L**2))*(-self.m_c*self.d*x_symbol[4]*x_symbol[3]+(self.L/self.R)*(u_symbol[0]-u_symbol[1]))) * timestep

  state_symbol_next = ca.vertcat(x_symbol_next, y_symbol_next, theta_symbol_next, v_symbol_next, omega_symbol_next)
  return ca.Function("differential_wheeled_dynamics", [x_symbol, u_symbol], [state_symbol_next])


 def forward_dynamics(self, x, u):
     '''
     Calculates the forward dynamics of the robot

     Args:
         x:   The vector of the robot linear and angular velocities ([x; y; theta; v; omega])
         u:   The vector of motor torques ([tau_r; tau_l])

     Returns:
         A 5D array of the updated state
     '''
     q    = np.zeros([self.n_dim,])
     q[0] = x[3]*np.cos(x[2])
     q[1] = x[3]*np.sin(x[2])
     q[2] = x[4]
     q[3] = (self.R**2/(self.m*self.R**2+2*self.I_w))*(self.m_c*self.d*x[4]**2+(1/self.R)*(u[0]+u[1]))
     q[4] = (self.R**2/(self.I*self.R**2+2*self.I_w*self.L**2))*(-self.m_c*self.d*x[4]*x[3]+(self.L/self.R)*(u[0]-u[1]))
     return q

 def plot(self, x, u, c, path, save_dir):
     '''
     This function plots the robot state and action 

     Args:
        x:   state of the robot as a 5D array dot ([x; y; theta; v; omega])
        u:   control input as a 2D array [tau_r; tau_l]
        save_dir:   the directory to be used for saving the animation
     Returns:
        None, it saves the generated plots in save_dir directory
     '''
     plt.figure(0)
     plt.plot(x[0,:]   , x[1,:],    'b', label='Robot')
     plt.plot(path[0,:], path[1,:], 'r', label='Ref')
     plt.plot(c[0,:], c[1,:], 'g*', label='Carrot')
     plt.xlabel('X')
     plt.ylabel('Y')
     plt.legend()
     plt.savefig(save_dir+'x_y.png')

     plt.figure(1)
     plt.plot(x[2,:])
     plt.ylabel('Phi')
     plt.savefig(save_dir+'phi.png')

     step = 10
     plt.figure(2)
#     if path.shape[0] == 3:
     plt.quiver(path[0,::step], path[1,::step], np.cos(path[2,::step]), np.sin(path[2,::step]), color='g')
 #    else:
  #      plt.plot(path[0,:], path[1,:], color='g')
     plt.xlabel('X')
     plt.ylabel('Y')
     plt.savefig(save_dir+'path.png')

 def animate_motion(self, x, u, save_dir):
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
     ax = fig.add_subplot(111, autoscale_on=False, xlim=[-0.3,2.3], ylim=[-0.3,2.3])
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

            x_a    = plotx[0,i]
            y_a    = plotx[1,i]
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
