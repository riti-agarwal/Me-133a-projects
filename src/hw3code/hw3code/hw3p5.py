'''
hw3p5.py

   This is a skeleton for HW3 Problem 5.  Please EDIT.

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
        #### PRECOMPUTE ANY DATA YOU MIGHT NEED.
        self.l1 = 1.0
        self.l2 = 1.0
        x = -0.8
        y = 1.0
        z = 1.4
        r = sqrt((x**2) + (y**2))
        theta_p_1 = atan2(-x/r, y/r)
        theta_2_pos = acos(((r**2) + (z**2) - (self.l1**2) - (self.l2**2))/ (2 * self.l1 * self.l2))
        theta_2_neg = -1 * theta_2_pos
        theta_1_pos_1 = atan2(r,z) - atan2((self.l2*sin(theta_2_pos)) , (self.l1 + (self.l2*cos(theta_2_pos))))
        theta_1_neg_1 = atan2(r,z) - atan2((self.l2*sin(theta_2_neg)) , (self.l1 + (self.l2*cos(theta_2_neg))))
        theta_p_2 = atan2(x/r, -y/r)
        theta_1_pos_2 = atan2(r,-z) - atan2((self.l2*sin(theta_2_pos)) , (self.l1 + (self.l2*cos(theta_2_pos))))
        theta_1_neg_2 = atan2(r,-z) - atan2((self.l2*sin(theta_2_neg)) , (self.l1 + (self.l2*cos(theta_2_neg))))
        
        self.solutions = [(theta_p_1, theta_1_pos_1, theta_2_pos), (theta_p_1, theta_1_neg_1, theta_2_neg), (theta_p_2, theta_1_pos_2, theta_2_pos), (theta_p_2, theta_1_neg_2, theta_2_neg)]
        
        print (self.solutions)
        


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, t, dt):
        #### COMPUTE THE POSITION AND VELOCITY VECTORS AS A FUNCTION OF TIME.
        def interp(start, end, end_time, time):
        	start_time = end_time - 1
        	#times = [start_time, end_time]
        	#positions = [self.solutions[start], self.solutions[end]]
        	#q = np.interp(time, times, positions)
        	#qdot = list(positions[1] - positions[0])
        	distance = np.asarray(self.solutions[end]) - np.asarray(self.solutions[start])
        	q = list(self.solutions[start] + (distance * (time - start_time)))
        	qdot = list(distance)
        	return (q, qdot)
        t = t % 6 
        if t <= 1:
        	return interp(0,1,1,t)
        elif t <= 1.5:
        	q = self.solutions[1]
        	qdot = [0.0]*3
        elif t <= 2.5:
        	return interp(1, 2, 2.5, t)
        elif t <= 3:
        	q = self.solutions[2]     
        	qdot = [0.0]*3
        elif t <= 4:
        	return interp(2, 3, 4, t)
        elif t <= 4.5:
        	q = self.solutions[3]
        	qdot = [0.0]*3
        elif t <= 5.5:
        	return interp (3, 0, 5.5, t)
        elif t <= 6:
        	q = self.solutions[0]
        	qdot = [0.0]*3
        else:
        	print('undefined t value')  
        # Return the position and velocity as python lists.
        return (q,qdot)


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
