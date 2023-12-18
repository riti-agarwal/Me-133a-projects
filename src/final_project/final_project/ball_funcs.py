import rclpy
import numpy as np 
import random

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray

from final_project.TransformHelpers     import *

class Ball(Node):
    # Initialization.
    def __init__(self, name, start, radius):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Initialize the ball position, velocity, set the acceleration.
        self.radius = radius
        self.side = 5.0 # distance between bounces
        
        # # self.init_p = np.array([0.0, 0.0, self.radius]).reshape((3,1))
        # # x,y,z random position
        # x_cord = random.uniform(self.radius, 0.75)
        # # x_cord = 0.7
        # # y_cord = random.uniform(self.radius, 1.5)
        # y_cord = 1.5
        # z_cord = random.uniform(self.radius, 1.0)
        # # z_cord = 0.2
        # self.init_p = np.array([x_cord, y_cord, z_cord]).reshape((3, 1))
        # # self.init_p = np.array([0.5, 1.0, 0.3]).reshape((3, 1))
        # # self.init_v = np.array([-1.0, -0.1,  5.0       ]).reshape((3,1))
        # self.init_v = np.array([0.0, -1.0, 0.0]).reshape(3, 1)

        # Specify the random target point
        x = random.uniform(0.4, 0.8)
        y = random.uniform(0.4, 0.8)
        z = random.uniform(0.2, 1.0)
        target_point = np.array([x, y, z]).reshape((3, 1))

        # Set random initial position
        x_cord = random.uniform(0.4, 2.0)
        y_cord = random.uniform(0.4, 2.0)
        z_cord = random.uniform(0.2, 2.0)
        self.init_p = np.array([x_cord, y_cord, z_cord]).reshape((3, 1))

        direction_vector = target_point - self.init_p
        random_velocity_magnitude = random.uniform(0.5, 1.5)
        self.init_v = (direction_vector / np.linalg.norm(direction_vector)) * random_velocity_magnitude

        self.p = self.init_p
        self.v = self.init_v
        # self.a = np.array([0.0, 0.0, -9.81      ]).reshape((3,1))
        self.a = np.array([0.0, 0.0, 0.0]).reshape((3, 1))
        
         # racket_collision_distance - how close ball needs to be to tennis racket for a collision to be detected.
        # used as a safety margin so a collision is detected even if the ball and racket are not perfectly aligned.
        self.racket_dist = 0.05

        # Create the sphere marker.
        diam        = 2 * self.radius
        self.marker = Marker()
        self.marker.header.frame_id  = "world"
        self.marker.header.stamp     = self.get_clock().now().to_msg()
        self.marker.action           = Marker.ADD
        self.marker.ns               = "point"
        self.marker.id               = 2
        self.marker.type             = Marker.SPHERE
        self.marker.pose.orientation = Quaternion()
        self.marker.pose.position    = Point_from_p(self.p)
        self.marker.scale            = Vector3(x = diam, y = diam, z = diam)
        self.marker.color            = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        # a = 0.8 is slightly transparent!

        # Create the marker array message.
        # self.mark = MarkerArray()
        # self.mark.markers.append(self.marker)

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        # self.dt    = 1.0 / float(rate)
        # self.t     = -self.dt
        self.start = start
        self.printed = False

        # Create a timer to keep calling update().
        # self.create_timer(self.dt, self.update)
        # self.get_logger().info("Running with dt of %f seconds (%fHz)" %
        #                        (self.dt, rate))
        print("ball init", self.init_p)

    # Shutdown
    def shutdown(self):
        # Destroy the node, including cleaning up the timer.
        self.marker.action = Marker.DELETE
        self.destroy_node()

    def get_random_path():
        # TODO
        return None

    def get_position(self):
        return self.p
    
    def get_direction(self):
        return get_direction_from_v(self.v)
    
    def get_pd_at_y(self, given_y = 0):
        # p = p0 + v0t + 1/2 a t^2
        # given_y = y0 + vy0 t
        t = ((given_y  + self.radius) - self.init_p[1, 0]) / self.init_v[1, 0]
        v = self.init_v + self.a * t
        d = get_direction_from_v(v)
        p = self.init_p + self.init_v * t + self.a * (t ** 2) / 2
        return p, d, t
    
    # Not considering acceleration
    def get_pd_at_dist(self, given_dist = 0.70, given_center = np.array([0.0, 0.0, 0.0]).reshape(-1, 1)):
        # p = p0 + v0t
        # dist(p - given_center) == given_dist
        # (p0[0] + v0[0] t - c[0])^2 + ... = gd ^ 2
        # (p0-c0)^2 + v0^2 t^2 + 2 (p0 - c0) (v0 t) = gd^2
        # (x- c)^2 ... = 0.4 ^ 2
        # 
        p1 = self.init_p
        if np.linalg.norm(self.init_p - given_center) < given_dist:
            return self.get_pd_at_y()
        p2 = self.init_p + self.init_v * 2.0
        min_d = np.linalg.norm(cross(p2 - p1, p1 - given_center))/np.linalg.norm(p2 - p1)
        # print("min d", min_d)
        a = (p1 - given_center)
        b = (p2 - p1)
        dot = a[0, 0] * b[0, 0] +  a[1, 0] * b[1, 0] + a[2, 0] * b[2, 0]
        td = - dot / (np.linalg.norm(p2 - p1) ** 2)
        p = p1 + td * (p2 - p1)
        if self.init_v[0, 0] != 0 and p[0, 0] - self.init_p[0, 0] != 0:
            t = (p[0, 0] - self.init_p[0, 0]) / self.init_v[0, 0]
        elif self.init_v[1, 0] != 0 and p[1, 0] - self.init_p[1, 0] != 0:
            t = (p[1, 0] - self.init_p[1, 0]) / self.init_v[1, 0]
        elif self.init_v[2, 0] != 0 and p[2, 0] - self.init_p[2, 0] != 0:
            t = (p[2, 0] - self.init_p[2, 0]) / self.init_v[2, 0]
        if min_d < given_dist:
            d = min_d
            dt = 1.0 / 1000
            while(d < given_dist):
                t -= dt
                p = self.init_p + self.init_v * t + self.a * (t ** 2) / 2
                d = np.linalg.norm(p - given_center)
            print("smaller", min_d, d, p, t)
        # TODO need to generalize
        v = self.init_v + self.a * t
        dir = get_direction_from_v(v)
        # p = self.init_p + self.init_v * t + self.a * (t ** 2) / 2
        print("min dist", p, dir, t, np.linalg.norm(p - given_center))
        return p, dir, t
        
        

    # Update - send a new joint command every time step.
    def update(self, t, dt, rac_p, rac_orientation_matrix, rac_radius, rac_length):
        # if self.p[1, 0] < 0.0:
        #     if not self.printed:
        #         print("ball p", self.p)
        #         self.printed = True
        #     # return None
        # if self.p[2, 0] < 0.0 or \
        if abs(self.p[0, 0]) + self.radius > self.side / 2.0 or  \
        abs(self.p[1, 0]) + self.radius > self.side / 2.0:
            if not self.printed:
                print("ball p", self.p)
                self.printed = True
            return None
        # Integrate the velocity, then the position.
        
        # Transform ball position to racket's local coordinates
        ball_p_local = rac_orientation_matrix.T @ (self.p - rac_p)

        # Calculate the closest point on the racket to the ball
        # z-axis should be nothing
        closest_point_on_racket_local = np.clip(ball_p_local, -rac_radius, rac_radius)
        closest_point_on_racket_local[2] = rac_length

        # Transform the closest point back to global coordinates
        closest_point_on_racket_global = rac_orientation_matrix @ closest_point_on_racket_local + rac_p
        if np.linalg.norm(self.p - closest_point_on_racket_global) < self.radius + rac_length:
            if not self.printed:
                print("ball p", self.p, "racket p", closest_point_on_racket_local)
                self.printed = True
            # return None
            self.v = rac_orientation_matrix @ (np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]])).reshape(3,3) @ rac_orientation_matrix.T @ self.v
            self.p = self.p + self.v * dt
            # print("collision")
            
        self.v += dt * self.a
        self.p += dt * self.v

        # can change this to0 check for collision
        # Check for a bounce - not the change in x velocity is non-physical.
        # if self.p[2,0] < self.radius:
        #     self.p[2,0] = self.radius + (self.radius - self.p[2,0])
        #     # changing the velocity in the z direction, so velocity is in the other direction
        #     self.v[2,0] *= -1.0
        #     print("collision1")
        #     # changing the velocity in the x direction
        #     self.v[0,0] *= -1.0   # Change x just for the fun of it!

        # Update the ID number to create a new ball and leave the
        # previous balls where they are.
        #####################
        # self.marker.id += 1
        #####################

        # Check for a bounce on the side wall
        # self.side is the width of the side wall
        # If a collision occurs, the ball's position is adjusted to be just outside the wall, 
        # and the x-axis component of the velocity is reversed to simulate a bounce.
        if abs(self.p[0, 0]) + self.radius > self.side / 2.0:
            # Bounce back from the side wall
            self.p[0, 0] = np.sign(self.p[0, 0]) * (self.side / 2.0 - self.radius)
            self.v[0, 0] *= -1.0
            # print("wall collision")
            
        if abs(self.p[1, 0]) + self.radius > self.side / 2.0:
            # Bounce back from the side wall
            self.p[1, 0] = np.sign(self.p[1, 0]) * (self.side / 2.0 - self.radius)
            self.v[1, 0] *= -1.0
            # print("wall collision")

        # Update the message and publish.
        now = self.start + Duration(seconds=t)
        self.marker.header.stamp  = now.to_msg()
        self.marker.pose.position = Point_from_p(self.p)
        # self.pub.publish(self.mark)


