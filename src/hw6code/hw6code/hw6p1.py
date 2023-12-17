import math
import numpy as np
M = np.array([[1,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,16]])
J = np.array([[1,-1,0,-1], [0,2,1,0]])
x_r_dot = np.array([[-1], [1]])
MJt = M @ np.transpose(J)
JMJt = J @ M @ np.transpose(J)
q_dot = MJt @ np.linalg.pinv(JMJt) @ x_r_dot 
print(q_dot)