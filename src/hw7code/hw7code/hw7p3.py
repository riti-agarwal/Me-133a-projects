import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from hw6code.GeneratorNode      import GeneratorNode
from hw6code.TransformHelpers   import *
from hw6code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw6code.KinematicChain     import KinematicChain

# Import the format for the condition number message
from std_msgs.msg import Float64


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        self.pleft  = np.array([0.3, 0.5, 0.15]).reshape((-1,1))
        self.pright = np.array([-0.3, 0.5, 0.15]).reshape((-1,1))

        # Initialize the current/starting joint position.
        self.q  = self.q0

        # FIXME: WHAT ELSE DO WE NEED TO INITIALIZE FOR THE INVERSE KINEMATICS?
        self.lam = 20

        # Setup up the condition number publisher
        self.pub = node.create_publisher(Float64, '/condition', 10)

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Decide which phase we are in:
        if t < 3.0:
            # Approach movement:
            (s0, s0dot) = goto(t, 3.0, 0.0, 1.0)
            pd = self.p0 + (self.pright - self.p0) * s0
            vd =           (self.pright - self.p0) * s0dot
            Rd = Reye()
            wd = np.zeros((3,1))
        else:
            t1 = (t-3) % 5.0
            if t1 < 1.25:
                (s0, s0dot) = goto(t1, 1.25, 0.0, 1.0)
                (pd, vd) = goto(t1, 1.25, self.pright, self.p0)
                (alpha, alphadot) = goto(t1, 1.25, 0, -(np.pi/2))
                Rd = Roty(alpha)
                wd = ey() * alphadot
            elif t1 < 2.5: 
                (s0, s0dot) = goto(t1 - 1.25, 1.25, 0.0, 1.0)
                (pd, vd) = goto(t1 - 1.25, 1.25, self.p0, self.pleft)
                (alpha, alphadot) = goto(t1 - 1.25, 1.25, 0, (np.pi/2))
                Rd = Roty(-np.pi/2) @ Rotz(alpha)
                print(Rd)
                wd = ez() * alphadot
            elif t1 < 3.75:
                (s0, s0dot) = goto(t1 - 2.5, 1.25, 0.0, 1.0)
                (pd, vd) = goto(t1 - 2.5, 1.25, self.pleft, self.p0)
                (alpha, alphadot) = goto(t1 - 2.5, 1.25, 0, -np.pi/2)
                Rd = Roty(-np.pi/2) @ Rotz(np.pi/2) @ Rotz(alpha)
                wd = ez() * alphadot
            elif t1 < 5:
                (s0, s0dot) = goto(t1 - 3.75, 1.25, 0.0, 1.0)
                (pd, vd) = goto(t1 - 3.75, 1.25, self.p0, self.pright)
                (alpha, alphadot) = goto(t1 - 3.75, 1.25, 0, (np.pi/2))
                Rd = Roty(-np.pi/2) @ Roty(alpha)
                wd = ey() * alphadot

        qprev = self.q
        (P, R, Jv, Jw) = self.chain.fkin(qprev)
        L = 0.4

        # Part a 
        # J = np.vstack((Jv, Jw))
        # Jbar = np.diag([1/L, 1/L, 1/L, 1, 1, 1]) @ J
        # condition = np.linalg.cond(Jbar)
        # V = np.vstack((vd, wd))
        # e = np.vstack((ep(pd, P), eR(Rd, R)))
        # qdot = np.linalg.pinv(J) @ (V + self.lam * e) 
        # q = qprev + dt * qdot
        # self.q = q
        
        # Part b
        # J = np.vstack((Jv, Jw, [1/2, 0, 1, 0, 0, 0, 0]))
        # Jbar = np.diag([1/L, 1/L, 1/L, 1, 1, 1, 1]) @ J
        # condition = np.linalg.cond(Jbar)
        # V = np.vstack((vd, wd, [0]))
        # e = np.vstack((ep(pd, P), eR(Rd, R), (0 - ((0.5*self.q[0]) + self.q[2]))))
        # qdot = np.linalg.pinv(J) @ (V + self.lam * e) 
        # q = qprev + dt * qdot
        # self.q = q

        # part c
        new_lam = 10
        q_desired = np.array([-np.pi/4, -np.pi/4, np.pi/2, -np.pi/2, 0, 0, 0]).reshape(7,1)
        qdot_secondary = new_lam * (q_desired - self.q)
        J = np.vstack((Jv, Jw))
        Jbar = np.diag([1/L, 1/L, 1/L, 1, 1, 1]) @ J
        condition = np.linalg.cond(Jbar)
        V = np.vstack((vd, wd))
        e = np.vstack((ep(pd, P), eR(Rd, R)))
        qdot_extra = (np.linalg.pinv(J) @ V) + (((np.identity(7) - (np.linalg.pinv(J) @ J))) @ qdot_secondary)
        qdot = np.linalg.pinv(J) @ (V + self.lam * e) + qdot_extra
        q = qprev + dt * qdot
        self.q = q

        # Part d 
        # J = np.vstack((Jv, Jw))
        # const = 0.5
        # cost_part2 = np.array([self.q[0][0], max(np.abs(self.q[0][0]), self.q[1][0]), 0, 0, 0, 0, 0]).reshape((7, 1))
        # cost = const * (1 / (self.q[0][0]**2 + self.q[1][0]**2)) * cost_part2
        # Jbar = np.diag([1/L, 1/L, 1/L, 1, 1, 1]) @ J
        # condition = np.linalg.cond(Jbar)
        # V = np.vstack((vd, wd))
        # e = np.vstack((ep(pd, P), eR(Rd, R)))
        # qdot = np.linalg.pinv(J) @ (V + self.lam * e) + ((np.identity(7) - (np.linalg.pinv(J) @ J)) @ cost)
        # q = qprev + dt * qdot
        # self.q = q

        #Publish the condition number.
        msg = Float64()
        msg.data = condition
        self.pub.publish(msg)



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


