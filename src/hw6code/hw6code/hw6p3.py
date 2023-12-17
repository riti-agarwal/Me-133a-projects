'''hw6p3.py

   This is the skeleton code for HW6 Problem 3.  Please EDIT.

   This creates a purely rotational movement.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

from turtle import goto
import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from hw6code.GeneratorNode      import GeneratorNode
from hw6code.TransformHelpers   import *
from hw6code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw6code.KinematicChain     import KinematicChain


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Initialize the current joint position to the starting
        # position and set the desired orientation to match.
        self.q = np.zeros((3,1))
        (_, self.Rd, _, _) = self.chain.fkin(self.q)

        # Pick the convergence bandwidth.
        self.lam = 20

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['pan', 'tilt', 'roll']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Choose the alpha/beta angles based on the phase.
        if t <= 2.0:
            # Part A (t<=2):
            (alpha, alphadot) = goto(t, 2.0, 0.0, (-np.pi/2))
            (beta,  betadot)  = (0.0, 0.0)
        else:
            # # To test part A only, you can return None and the node will exit.
            # return None

            # Part B (t>2):
            (alpha, alphadot) = ((-np.pi/2), 0.0)
            (beta,  betadot)  = t - 3.0 + np.exp(2.0 - t), 1.0 - np.exp(2.0 - t)

        
        # Compute the desired rotation and angular velocity.
        Wd_hat = np.array([[np.sqrt(2)/2], [np.sqrt(2)/2], [0.0]])
        wd = Wd_hat * betadot
        wd_new = Roty(-alpha) @ Wd_hat
        self.Rd = Roty(alpha) @ Rote(wd_new, beta)
        qprev = self.q
        

        # Grab the stored information from last cycle.

        # Compute the old forward kinematics.
        (p, R, Jv, Jw) = self.chain.fkin(qprev)
        error_r = eR(self.Rd, R)

        # Compute the inverse kinematics
        qdot = (np.linalg.inv(Jw)) @ (wd + (self.lam * error_r))

        # Integrate the joint position.
        q = qprev + dt * qdot
        self.q = q

        # Save the data needed next cycle.
        (_, self.Rd, _, _) = self.chain.fkin(self.q)

        # Return the position and velocity as python lists.
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
