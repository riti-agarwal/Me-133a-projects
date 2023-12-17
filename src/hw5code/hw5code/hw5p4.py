'''hw5p4.py

   This is skeleton code for HW5 Problem 4.  Please EDIT.

   This moves the tip in a straight line (tip spline), then returns in
   a joint spline.

'''

import rclpy
import numpy as np

from math                       import pi, sin, cos, acos, atan2, sqrt, fmod

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TrajectoryUtils    import goto, spline, goto5, spline5

# Grab the fkin and Jac from P1.
from hw5code.hw5p1              import fkin, Jac


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Define the known tip/joint positions.
        self.qA = np.radians(np.array([ 0, 60, -120])).reshape(3,1)
        self.xA = fkin(self.qA)

        self.qD = None
        self.xD = np.array([0.5, -0.5, 1.0]).reshape(3,1)

        # Select the leg duration.
        self.T = 3.0

        # Initialize the parameters and anything stored between cycles!
        self.lambd = 20  # Set lambda to 20.
        self.q_prev = self.qA
        self.x_prev = self.xA

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, t, dt):

        # x_tilde = actual x_prev - supposed to be x_prev(fkin(q_prev))
        # xdot = spline (xA, xd)
        # q = (qdot * dt) + q_prev
        
        # First modulo the time by 2 legs.
        t = fmod(t, 2*self.T)

        if t <= self.T:
            x_tilde = self.x_prev - fkin(self.q_prev)
            _, x_dot = spline(t, self.T, self.xA, self.xD, 0, 0)
            J = Jac(self.q_prev)
            qdot = np.linalg.inv(J.astype('float64')) @ (x_dot + (self.lambd*x_tilde))
            q = (qdot*dt) + self.q_prev
            self.q_prev = q
            self.x_prev = fkin(q)
        else: 
            if (self.qD is None):
                self.qD = self.q_prev
            q, qdot = spline(t-self.T, self.T, self.qD, self.qA, 0, 0)
            self.q_prev = q
            self.x_prev = fkin(q)

        # Return the position and velocity as python lists!
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
