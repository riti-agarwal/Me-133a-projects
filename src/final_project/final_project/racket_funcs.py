from enum import Enum
import rclpy
import numpy as np
import math 

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp, radians

# Grab the utilities
from final_project.GeneratorNode      import GeneratorNode
from final_project.TransformHelpers   import *
from final_project.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from final_project.KinematicChain     import KinematicChain

class state(Enum):
    TOTARGET = 1
    WAITINGTARGET = 2
    TOINIT = 3
    WAITINGINIT = 4

class Racket():
    def __init__(self, node, ball_period, goal = None):
        self.goal = goal
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        
        # Define the various points.
        self.q0 = np.radians(np.array([0, 90, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        # self.R0 = Reye() @ Rotx(-pi/2) @ Roty(-pi/2)
        self.R0 = Reye()
        
        self.p_prev = self.p0
        self.R_prev = self.R0
        
        # TODO from ball trajectory, get position and normal orientation
        self.p_target = np.array([0.5, 0.5, 0.15]).reshape((-1,1))
        self.r_target = Reye() @ Rotx(-pi/2) @ Roty(-pi/2)
        
        self.target_changed = False
        self.ball_hit = False
        self.ball_period = ball_period
        
        self.duration = 0.5
        self.last_time = 0
        self.state = state.WAITINGINIT

        self.rac_radius = 0.1
        self.rac_length = 0.01
        self.lamb = 50
        self.q  = self.q0
        self.p = self.p0
        self.R = self.R0
        
        self.shoulder = np.array([0.0, 0.0, 0.6]).reshape((-1, 1))
        self.link_dist = 0.8

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']
    
    def set_racket_target(self, ball, time):
        # ball_p, ball_d, t = ball.get_pd_at_y(given_y = 0)
        ball_p, ball_d, t = ball.get_pd_at_dist()
        self.p_target = ball_p
        if self.goal is None:
            self.r_target = Rotx(ball_d[0, 0]) @ Roty(ball_d[1, 0]) @ Rotz(ball_d[2, 0])
        else:
            to_goal = get_direction_from_v(ball_p - self.goal)
            des_z = (ball_d + to_goal) / 2
            if np.linalg.norm(des_z) == 0:
                des_z = -ey()
            # des_z = np.array([1.0, -1.0, 0.0]).reshape((3,1))
            # print("z", des_z, ball_d, to_goal)
            des_z = get_direction_from_v(des_z)
            
            if np.linalg.norm(self.shoulder - ball_p) > self.link_dist * 0.8:
                print("norm")
                des_y = -get_direction_from_v(self.shoulder - ball_p)
                des_x = get_direction_from_v(cross(des_y, des_z))
            else:
                des_x = get_direction_from_v(self.R @ ex())
            # des_x = get_direction_from_v(np.array([0.5, 0.5, 0]).reshape((3, 1)))
            des_y = get_direction_from_v(cross(des_z, des_x))
            des_x = get_direction_from_v(cross(des_y, des_z))
            self.r_target = Rot_from_xyz(x=des_x, y =des_y, z=des_z)
            # self.r_target = Rotx(r_vec[0, 0]) @ Roty(r_vec[1, 0]) @ Rotz(r_vec[2, 0])
            # self.p_target = self.p_target + pe(des_z, ball.radius)
                            # + pe(des_y, ball.radius) \
                            # + pe(des_x, ball.radius)
            # self.r_target = Reye() @ Rotx(pi/2) @ Rotz(-pi/2)
        # TODO within ball trajectory
        self.target_changed = True
        self.duration = t * (0.70)
        # self.duration = 2.5
        self.checkwaiting(time)
        
        print("target", self.p_target, self.r_target, t)
    
    def checkwaiting(self, t):
        if self.state == state.WAITINGINIT and self.target_changed:
            self.state = state.TOTARGET
            self.last_time = t
            self.target_changed = False
            self.p_prev = self.p
            self.R_prev = self.R
        elif self.state == state.WAITINGTARGET and self.ball_hit:
            self.state = state.WAITINGINIT
            # self.state = state.TOINIT
            self.last_time = t
            self.ball_hit = False
            self.duration = self.ball_period
            self.p_prev = self.p
            self.R_prev = self.R
            # self.state = state.WAITINGTARGET
        # print(self.state)
            
    def get_position(self):
        return self.p

    def get_orientation(self):
        return self.R
    
    def set_goal(self, goal):
        self.goal = goal

    def get_radius(self):
        return self.rac_radius
    
    def get_length(self):
        return self.rac_length

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if self.state == state.WAITINGINIT or self.state == state.WAITINGTARGET:
            q = self.q
            qdot = np.zeros((6, 1))
            self.checkwaiting(t)
        else:
            if self.state == state.TOTARGET:
                if t - self.last_time > self.duration:
                    print("at target", self.p, self.R)
                    self.last_time = t
                    q = self.q
                    qdot = np.zeros((6, 1))
                    self.state = state.WAITINGTARGET
                    return (q.flatten().tolist(), qdot.flatten().tolist())
                p0 = self.p_prev
                pf = self.p_target
                r0 = self.R_prev
                rf = self.r_target
            elif self.state == state.TOINIT:
                if t - self.last_time > self.duration:
                    self.last_time = t
                    q = self.q
                    qdot = np.zeros((6, 1))
                    self.state = state.WAITINGINIT
                    return (q.flatten().tolist(), qdot.flatten().tolist())
                p0 = self.p_target
                pf = self.p0
                r0 = self.r_target
                rf = self.R0
                

            # working kinematics over specfified 
            # t = fmod(t - self.last_time, self.duration)  
            # # t = t - self.last_time
            # e = ex() + ey() + ez()
            # alpha = pi / 2
            
            # (s0, s0dot) = goto(t, self.duration, 0.0, 1.0)

            # pd = p0 + (pf - p0) * s0
            # vd =      (p0 - pf) * s0dot

            # Rd = r0 @ (s0 * np.linalg.inv(r0)) @ rf
            # alphadot = alpha * s0dot
            # wd = alphadot * e

            # qlast = self.q
            # (p, R, Jv, Jw) = self.chain.fkin(qlast)
            # J = np.vstack((Jv, Jw))
            # V = np.vstack((vd, wd))
            # E = np.vstack((ep(pd, p), eR(Rd, R)))

            # START WORKING HERE

            t = fmod(t - self.last_time, self.duration)  
            # alpha = pi / 2
            
            (s0, s0dot) = goto(t, self.duration, 0.0, 1.0)

            pd = p0 + (pf - p0) * s0
            vd =      (pf - p0) * s0dot

            # alphadot = alpha * s0dot

            qlast = self.q
            Rlast = self.R
            nf = (rf[:, [-1]])
            print("nf is", nf.T)
            n0 = (r0[:, [-1]])
            print("n0 is", n0.T)
            a = cross(n0, nf).reshape(3,1)
            axis = a / np.linalg.norm(a)
            # theta_f = np.arcsin(np.linalg.norm(a))
            theta_f = np.arccos(n0.T @ nf)
            # wd has to be 3,1 matrix, and i drop the last row? 
            wd = axis * (theta_f * s0dot)
            nd = (Rote(axis, (theta_f * s0))) @ n0
            print("nd is", nd.T)
            (p, R, Jv, Jw) = self.chain.fkin(qlast)
            wd = R.T @ wd
            wd = wd[:-1, :]
            Jw_padel = R.T @ Jw
            # Here I am dropping the last row. 
            Jw_padel = Jw_padel[:-1, :]
            J = np.vstack((Jv, Jw_padel))
            V = np.vstack((vd, wd))
            n = (R[:, [-1]])
            # print("n is", n)
            # does en also lose its last row?
            en = cross(nd, n).reshape(3,1)
            en = R.T @ en
            en = en[:-1, :]
            E = np.vstack((ep(pd, p), en))
            # print(V.shape)
            # print(E.shape)


            # This is with multiple singularities
            # qdot = np.linalg.pinv(J) @ (V + self.lamb * E)

            # This is with smoother singularities, no wanted position of the arm
            weight = 0.05
            Jwinv = J.T @ np.linalg.pinv(J @ J.T + weight**2 * np.eye(5))
            qdot = Jwinv @ (V + self.lamb * E)

            # Range: theta1: does not matter
            # theta2: -50 to 75
            # Theta3: 34 to 130 degrees 
            # theta4: does not matter
            # theta5: -20 to 20 
            # theta6: -10 to 42 degrees

            # weight = 0.1

            # Jwinv = np.linalg.pinv((np.transpose(J) @ J) + (weight**2 * np.identity(6))) @ np.transpose(J)
            # q_desired = np.array([0, -math.radians(30), math.radians(30), 0, 0, 0]).reshape(6,1)
            # qlast_modified = np.array([0, qlast[1][0], qlast[2][0], 0, 0, 0]).reshape(6,1)
            # new_lam = 20
            # qdot_secondary = new_lam * (q_desired - qlast)
            # qdot_extra = (((np.identity(6) - (Jwinv @ J))) @ qdot_secondary)
            # qdot = Jwinv @ (V + self.lamb * E) + qdot_extra

            # Part (c): secondary task as goal
            lams = 50.0
            qc = np.radians(np.array([0,-30,30,0,0,0]).reshape((6,1)))
            qlast_modified = np.array([0, 0, 0, qlast[3][0], qlast[4][0], qlast[5][0]]).reshape(6,1)
            qsdot = lams * (qc - qlast)
            qsdot[0:3,0] = 0
            qdot_extra = (((np.identity(6) - (Jwinv @ J))) @ qsdot)
            qdot = Jwinv @ (V + self.lamb * E) + qdot_extra

            
            q = qlast + dt * qdot
            
            # Update
            self.q = q
            self.p = p
            self.R = R

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())