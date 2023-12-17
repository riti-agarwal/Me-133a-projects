'''hw6p5.py

   This is the skeleton code for HW6 Problem 5.  Please EDIT.

   This uses the inverse kinematics from Problem 4, but adds a more
   complex trajectory.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np
from enum import Enum

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain

# from GeneratorNode      import GeneratorNode
# from TransformHelpers   import *
# from TrajectoryUtils    import *

# # Grab the general fkin from HW5 P5.
# from KinematicChain     import KinematicChain

class state(Enum):
    TOTARGET = 1
    WAITINGTARGET = 2
    TOINIT = 3
    WAITINGINIT = 4

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        
        # Define the various points.
        self.q0 = np.radians(np.array([0, 90, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()
        
        # TODO from ball trajectory, get position and normal orientation
        self.p_target = np.array([0.3, 0.5, 0.15]).reshape((-1,1))
        self.r_target = Reye() @ Rotx(-pi/2) @ Roty(-pi/2)
        
        self.duration = 2.5
        self.last_time = 0
        self.state = state.TOTARGET

        self.lamb = 20
        self.q  = self.q0

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']
    
    def gettarget(self):
        # TODO
        # calculate normal vector
        return None
    
    def checkwaiting(self, t):
        if self.state == state.WAITINGINIT:
            # if target changed
            self.state = state.TOTARGET
            self.last_time = t
        elif self.state == state.WAITINGTARGET:
            # if ball hit
            self.state = state.TOINIT
            self.last_time = t

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if self.state == state.WAITINGINIT or self.state == state.WAITINGTARGET:
            q = self.q
            qdot = np.zeros((6, 1))
            self.checkwaiting(t)
        else:
            if self.state == state.TOTARGET:
                if t - self.last_time >= self.duration:
                    self.last_time = t
                    q = self.q
                    qdot = np.zeros((6, 1))
                    self.state = state.WAITINGTARGET
                    return (q.flatten().tolist(), qdot.flatten().tolist())
                p0 = self.p0
                pf = self.p_target
                r0 = self.R0
                rf = self.r_target
            elif self.state == state.TOINIT:
                if t - self.last_time >= self.duration:
                    self.last_time = t
                    q = self.q
                    qdot = np.zeros((6, 1))
                    self.state = state.WAITINGINIT
                    return (q.flatten().tolist(), qdot.flatten().tolist())
                p0 = self.p_target
                pf = self.p0
                r0 = self.r_target
                rf = self.R0
                
            t = fmod(t - self.last_time, self.duration)  
            e = ex() + ey() + ez()
            alpha = pi / 2
            
            (s0, s0dot) = goto(t, 2.5, 0.0, 1.0)

            pd = p0 + (pf - p0) * s0
            vd =      (p0 - pf) * s0dot

            Rd = r0 @ (s0 * np.linalg.inv(r0)) @ rf
            alphadot = alpha * s0dot
            wd = alphadot * e

            qlast = self.q
            (p, R, Jv, Jw) = self.chain.fkin(qlast)
            J = np.vstack((Jv, Jw))
            V = np.vstack((vd, wd))
            E = np.vstack((ep(pd, p), eR(Rd, R)))
            
            qdot = np.linalg.pinv(J) @ (V + self.lamb * E)
            
            q = qlast + dt * qdot
            
            # Update
            self.q = q
            
            # TODO Add secondary tasks 

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
