import numpy as np
def straightLine(p1, p2, horizon):
 '''
 This function generates a path between 2 points
 
 Args:
  p1: first  3D point of the line
  p2: second 3D point of the line
  horizon: full horizon of the path

 Returns:
  an array of the way points
 '''
 path      = np.zeros([p1.shape[0],horizon])
 ds        = (p2-p1)/horizon
 path[:,0] = p1
 for i in range(1,horizon):
  path[:,i] = path[:,i-1] + ds
 return path
