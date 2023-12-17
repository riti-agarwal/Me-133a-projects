'''
hw3p4.py

   This is a skeleton for HW3 Problem 4.  Please EDIT.

   It creates a trajectory generation node to command the joint
   movements.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState
'''

import rclpy
import numpy as np

from math               import pi, sin, cos, acos, atan2, sqrt, fmod

from rclpy.node         import Node
from sensor_msgs.msg    import JointState


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self):
        pass

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names MATCHING THE JOINT NAMES IN THE URDF!
        return ['pan', 'tilt']

    # Evaluate at the given time.
    def evaluate(self, t, dt):
        # Compute the joint values.  We could build up numpy arrays as
        # position and velocity vectors, but just compute the values:
        theta_pan  = (np.pi/3)* np.sin(2*t) 
        omega_pan  = (2*np.pi/3)*np.cos(2*t)

        theta_tilt =  ((np.pi/3)*np.sin(t)) - ((np.pi/9)*np.cos(6*t))
        omega_tilt =  (np.pi/3*cos(t)) + ((2*np.pi/3)*np.sin(6*t))
        # Return the position and velocity as python lists.
        return ([theta_pan, theta_tilt], [omega_pan, omega_tilt])


#
#   Generator Node Class
#
#   This inherits all the standard ROS node stuff, but adds an
#   update() method to be called regularly by an internal timer and a
#   shutdown method to stop the timer.
#
#   Take the node name and the update frequency as arguments.
#
class Generator(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it 'generator'
        super().__init__(name)

        # Set up the trajectory.
        self.trajectory = Trajectory()
        self.jointnames = self.trajectory.jointnames()

        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Create a timer to trigger calculating/sending commands.
        self.timer     = self.create_timer(1/float(rate), self.update)
        self.dt        = self.timer.timer_period_ns * 1e-9
        self.t         = - self.dt
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()

    # Update - send a new joint command every time step.
    def update(self):
        # Grab the current time (from the ROS clock, since 1970).
        now = self.get_clock().now()

        # To avoid any time jitter enforce a constant time step in
        # integrate to get the current time.
        self.t += self.dt

        # Compute the desired joint positions and velocities for this time.
        (q, qdot) = self.trajectory.evaluate(self.t, self.dt)

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = now.to_msg()      # Current time for ROS
        cmdmsg.name         = self.jointnames   # List of joint names
        cmdmsg.position     = q                 # List of joint positions
        cmdmsg.velocity     = qdot              # List of joint velocities
        self.pub.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates.
    generator = Generator('generator', 100)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted.
    rclpy.spin(generator)

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
