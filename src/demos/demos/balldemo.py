"""balldemo.py

   Simulate a non-physical ball and publish as a visualization marker
   array to RVIZ.

   Node:        /balldemo
   Publish:     /visualization_marker_array     visualization_msgs/MarkerArray

"""

import rclpy
import numpy as np

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray

from demos.TransformHelpers     import *


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)

        # Initialize the ball position, velocity, set the acceleration.
        self.radius = 0.033
        self.side = 2.0 # distance between bounces

        self.p = np.array([0.0, 0.0, self.radius]).reshape((3,1))
        self.v = np.array([1.0, 0.1,  5.0       ]).reshape((3,1))
        self.a = np.array([0.0, 0.0, -9.81      ]).reshape((3,1))

        # Create the sphere marker.
        diam        = 2 * self.radius
        self.marker = Marker()
        self.marker.header.frame_id  = "world"
        self.marker.header.stamp     = self.get_clock().now().to_msg()
        self.marker.action           = Marker.ADD
        self.marker.ns               = "point"
        self.marker.id               = 1
        self.marker.type             = Marker.SPHERE
        self.marker.pose.orientation = Quaternion()
        self.marker.pose.position    = Point_from_p(self.p)
        self.marker.scale            = Vector3(x = diam, y = diam, z = diam)
        self.marker.color            = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        # a = 0.8 is slightly transparent!

        # Create the marker array message.
        self.mark = MarkerArray()
        self.mark.markers.append(self.marker)

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Create a timer to keep calling update().
        self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

    # Shutdown
    def shutdown(self):
        # Destroy the node, including cleaning up the timer.
        self.destroy_node()

    # Update - send a new joint command every time step.
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt

        # Integrate the velocity, then the position.
        self.v += self.dt * self.a
        self.p += self.dt * self.v

        #X-Axis:
        # Positive X-axis points to the right.
        # Negative X-axis points to the left.
        # Y-Axis:
        # Positive Y-axis points forward.
        # Negative Y-axis points backward.
        # Z-Axis:
        # Positive Z-axis points upward.
        # Negative Z-axis points downward.


        # can change this to0 check for collision
        # Check for a bounce - not the change in x velocity is non-physical.
        # if z axis ('up-down') is less than 0.1, it creates a bounce. 
        if self.p[2,0] < self.radius:
            self.p[2,0] = self.radius + (self.radius - self.p[2,0])
            # changing the velocity in the z direction, so velocity changes from going up to going down or vice versa
            self.v[2,0] *= -1.0
            # changing the velocity in the x direction
            # self.v[0,0] *= -1.0   # x velocity changes from left to right or right to left when bounce happens


        # Check for a bounce on the side wall
        # self.side is the width of the side wall
        # If a collision occurs, the ball's position is adjusted to be just outside the wall, 
        # and the x-axis component of the velocity is reversed to simulate a bounce.
        if abs(self.p[0, 0]) + self.radius > self.side / 2.0:
            print("hello world")
            # Bounce back from the side wall
            self.p[0, 0] = np.sign(self.p[0, 0]) * (self.side / 2.0 - self.radius)
            self.v[0, 0] *= -1.0


        # FOR RACQUET COLLISION - NEEDS POSITION OF TENNIS RACQUET

        # # racket_collision_distance- how close ball needs to be to tennis racket for a collision to be detected. 
        # # used as safety margin so collision is detected even if the ball and racket are not perfectly aligned.
        # racket_collision_distance = 0.05  
        # CHANGE direction_to_racket by also using the orientation of the racket
        # if np.linalg.norm(self.p - self.rac_p) < self.radius + racket_collision_distance:
        #     # Bounce back from the tennis racket
        #     direction_to_racket = (self.rac_p - self.p) / np.linalg.norm(self.rac_p - self.p)
        #     self.p = self.rac_p - (self.radius + racket_collision_distance) * direction_to_racket
        #     self.v = -self.v + 2 * np.dot(self.v.T, direction_to_racket) * direction_to_racket

        

        # print(self.p)

        # Update the message and publish.
        now = self.start + Duration(seconds=self.t)
        self.marker.header.stamp  = now.to_msg()
        self.marker.pose.position = Point_from_p(self.p)
        self.pub.publish(self.mark)



#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)
    node = DemoNode('balldemo', 100)

    # Run until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
