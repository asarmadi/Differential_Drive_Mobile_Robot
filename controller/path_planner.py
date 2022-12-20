import numpy as np
def straightLine(p1, p2, horizon):
 '''
 This function generates a path between 2 points
 
 Args:
  p1: first  5D point of the line
  p2: second 5D point of the line
  horizon: full horizon of the path

 Returns:
  an array of the way points
 '''
 path      = np.zeros([p1.shape[0],horizon])
 ds        = (p2-p1)/horizon
 theta     = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
 path[:,0] = p1
 path[3,0] = theta
 for i in range(1,horizon):
  path[0:2,i] = path[0:2,i-1] + ds[0:2]
  path[2,i]   = theta
 return path
