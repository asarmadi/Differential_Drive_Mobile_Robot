import matplotlib
import numpy as np
from tqdm import tqdm
import casadi as ca
from models.robot import Robot
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Quad2D(Robot):
 def __init__(self, dt):
    super().__init__(dt)
    self.m          =  0.6            # mass of the robot without wheels
    self.I          =  0.15           # moment of Inertia of the robot without wheels moment of Inertia
    self.r          =  0.2            # distance from center of wheels to the CoM

    self.l          =  0.2            # half of height of the robot
    self.g          =  9.8            # gravity 

    self.n_dim      =  6              # number of state dimensions

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

  x_symbol_next     = x_symbol[0] + x_symbol[3] * timestep
  y_symbol_next     = x_symbol[1] + x_symbol[4] * timestep
  theta_symbol_next = x_symbol[2] + x_symbol[5] * timestep
  vx_symbol_next    = x_symbol[3] + ((-np.sin(x_symbol[3])*(u_symbol[0]+u_symbol[1])) / self.m) * timestep
  vy_symbol_next    = x_symbol[4] + ((np.cos(x_symbol[3])*(u_symbol[0]+u_symbol[1]) / self.m) - self.g) * timestep
  omega_symbol_next = x_symbol[5] + ((self.r/self.I)*(u_symbol[0]-u_symbol[1])) * timestep

  state_symbol_next = ca.vertcat(x_symbol_next, y_symbol_next, theta_symbol_next, vx_symbol_next, vy_symbol_next, omega_symbol_next)
  return ca.Function("differential_wheeled_dynamics", [x_symbol, u_symbol], [state_symbol_next])


 def forward_dynamics(self, x, u):
     '''
     Calculates the forward dynamics of the robot

     Args:
         x:   The vector of the robot linear and angular velocities ([x; y; theta; vx; vy; omega])
         u:   The vector of motor torques ([u_1; u_2])

     Returns:
         A 6D array of the updated state
     '''
     q    = np.zeros([self.n_dim,])
     q[0] = x[3]
     q[1] = x[4]
     q[2] = x[5]
     q[3] = (-np.sin(x[3])*(u[0]+u[1])) / self.m
     q[4] = (np.cos(x[3])*(u[0]+u[1]) / self.m) - self.g
     q[5] = (self.r/self.I)*(u[0]-u[1])
     return q

 def plot(self, x, u, c, path, save_dir):
     '''
     This function plots the robot state and action 

     Args:
        x:   state of the robot as a 5D array dot ([x; vx; y; vy; theta; omega])
        u:   control input as a 2D array [u_1; u_2]
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
     plotu = u[:,::steps]

     fig = matplotlib.figure.Figure(figsize=[6,6])
     ax = fig.add_subplot(111, autoscale_on=False, xlim=[np.min(x[0,:])-0.3,np.max(x[0,:])+0.3], ylim=[np.min(x[1,:])-0.3,np.max(x[1,:])+0.3])
#     ax = fig.add_subplot(111, autoscale_on=False, xlim=[-0.3,2.3], ylim=[-0.3,2.3])
     ax.grid()

     list_of_lines = []

     line, = ax.plot([], [], 'k', lw=6)
     list_of_lines.append(line)
     # the left propeller
     line, = ax.plot([], [], 'b', lw=4)
     list_of_lines.append(line)
     # the right propeller
     line, = ax.plot([], [], 'b', lw=4)
     list_of_lines.append(line)
     # the left thrust
     line, = ax.plot([], [], 'r', lw=1)
     list_of_lines.append(line)
     # the right thrust
     line, = ax.plot([], [], 'r', lw=1)
     list_of_lines.append(line)

     def animate(i):
            for l in list_of_lines: #reset all lines
                l.set_data([],[])

            theta = plotx[2,i]
            x     = plotx[0,i]
            y     = plotx[1,i]
            trans = np.array([[x,x],[y,y]])
            rot   = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

            main_frame = np.array([[-self.l, self.l], [0,0]])
            main_frame = rot @ main_frame + trans

            left_propeller = np.array([[-1.3 * self.l, -0.7*self.l], [0.1,0.1]])
            left_propeller = rot @ left_propeller + trans

            right_propeller = np.array([[1.3 * self.l, 0.7*self.l], [0.1,0.1]])
            right_propeller = rot @ right_propeller + trans

            left_thrust = np.array([[self.l, self.l], [0.1, 0.1+plotu[0,i]*0.04]])
            left_thrust = rot @ left_thrust + trans

            right_thrust = np.array([[-self.l, -self.l], [0.1, 0.1+plotu[0,i]*0.04]])
            right_thrust = rot @ right_thrust + trans

            list_of_lines[0].set_data(main_frame[0,:], main_frame[1,:])
            list_of_lines[1].set_data(left_propeller[0,:], left_propeller[1,:])
            list_of_lines[2].set_data(right_propeller[0,:], right_propeller[1,:])
            list_of_lines[3].set_data(left_thrust[0,:], left_thrust[1,:])
            list_of_lines[4].set_data(right_thrust[0,:], right_thrust[1,:])


            return list_of_lines

     def init():
         return animate(0)

     ani = animation.FuncAnimation(fig, animate, np.arange(0, len(plotx[0,:])),
         interval=use_dt, blit=True, init_func=init)

     f = save_dir
     writervideo = animation.FFMpegWriter(fps=60)
     ani.save(f, writer=writervideo)

     plt.close(fig)
