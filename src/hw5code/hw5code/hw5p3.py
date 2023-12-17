'''hw5p3.py

   This is skeleton code for HW5 Problem 3.  Please EDIT.

   Repeatedly and smoothly move the 3DOF.

'''

import rclpy
import numpy as np

from math                       import pi, sin, cos, acos, atan2, sqrt, fmod

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TrajectoryUtils    import goto, spline, goto5, spline5


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Define the three joint positions.
        self.qA = np.radians(np.array([   0,  60, -120])).reshape(3,1)
        self.qB = np.radians(np.array([ -90, 135,  -90])).reshape(3,1)
        self.qC = np.radians(np.array([-180,  60, -120])).reshape(3,1)

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, t, dt):
        # First modulo the time by 4 seconds
        t = fmod(t, 6.0)

        # Compute the joint values.
        if   (t < 2.0): (q, qdot) = spline(t    , 2.0, self.qA, self.qB, 0, (np.array([-1.17809725,0,0])).reshape(3,1)) 
        elif (t < 4.0): (q, qdot) = spline(t-2.0, 2.0, self.qB, self.qC, (np.array([-1.17809725,0,0])).reshape(3,1), 0) 
        else:           (q, qdot) = spline(t-4.0, 2.0, self.qC, self.qA, 0, 0)

        # # Compute the joint values.
        # if   (t < 2.0): (q, qdot) = goto(t    , 2.0, self.qA, self.qB) 
        # elif (t < 4.0): (q, qdot) = goto(t-2.0, 2.0, self.qB, self.qC) 
        # else:           (q, qdot) = goto(t-4.0, 2.0, self.qC, self.qA)

        # Return the position and velocity as flat python lists!
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
