'''hw5p2.py

   This is skeleton code for HW5 Problem 2.  Please EDIT.

   Implement the Newton-Raphson for seven target points.

'''

import numpy as np

# Grab the fkin and Jac from P1.
from hw5p1 import fkin, Jac
import matplotlib.pyplot as plt

#
#  Utilities
#
# 360 deg wrapping:
def wraps(q):
    return np.round(q / (2*np.pi))

def unwrapped(q):
    return q - np.round(q / (2*np.pi)) * (2*np.pi)

# 3 DOF Multiplicities - return True of False!
def elbow_up(q):
    assert np.shape(q) == (3,1), "Requires 3-element column vector"
    return np.sin(q[2,0]) < 0.0

def front_side(q):
    assert np.shape(q) == (3,1), "Requires 3-element column vector"
    return l1 * np.cos(q[1,0]) + l2 * np.cos(q[1,0] + q[2,0]) > 0.0



#
#  Newton Raphson
#
def newton_raphson(xgoal):
    # Collect the distance to goal and change in q every step!
    xdistance = []
    qstepsize = []

    # Set the initial joint value guess.
    q = np.array([0.0, np.pi/2, -np.pi/2]).reshape(3,1)

    # IMPLEMENT THE NEWTON-RAPHSON ALGORITHM!
    max_iterations = 20  # Maximum number of iterations
    epsilon = 10 ^ (-12)  # Convergence threshold

    q_store = []
    wraps_store = []
    up_store = []
    front_store = []
    count = 0 

    for i in range(max_iterations):
        J = Jac(q)
        x = fkin(q)
        delta_q = np.linalg.inv(J.astype(float)).dot((xgoal - x).astype(float))
        q = q + delta_q
        distance_to_goal = np.linalg.norm(xgoal - x)
        xdistance.append(distance_to_goal)
        qstepsize.append(np.linalg.norm(delta_q))
        wraps_store.append(wraps(q))
        q_store.append(q)
        up_store.append(elbow_up(q))
        front_store.append(front_side(q))
        count += 1

        # Check for convergence
        if distance_to_goal < epsilon:
            break
    print (count)
        

    # Create a plot of x distances to goal and q step sizes, for N steps.
    N = 20
    xdistance = xdistance[:N+1]
    qstepsize = qstepsize[:N+1]

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    
    ax1.plot(range(len(xdistance)), xdistance)
    ax2.plot(range(len(qstepsize)), qstepsize)

    ax1.set_title(f'Convergence Data for {xgoal.T}')
    ax2.set_xlabel('Iteration')

    ax1.set_ylabel('Task Distance to Goal')
    ax1.set_ylim([0, max(xdistance)])
    ax1.set_xlim([0, N])
    ax1.set_xticks(range(N+1))
    ax1.grid()

    ax2.set_ylabel('Joint Step Size')
    ax2.set_ylim([0, max(qstepsize)])
    ax2.set_xlim([0, N])
    ax2.set_xticks(range(N+1))
    ax2.grid()

    plt.show()

    print(wraps_store[-1])
    print(up_store[-1])
    print(front_store[-1])
    print("q is: ", q_store[-1])


#
#  Main Code
#
def main():
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)

    # Prcess each target (goal position).
    for xgoal in [np.array([0.5,  1.0, 0.5]).reshape((3,1)), 
                  np.array([1.0,  0.5, 0.5]).reshape((3,1)),
                  np.array([2.0,  0.5, 0.5]).reshape((3,1)),
                  np.array([0.0, -1.0, 0.5]).reshape((3,1)),
                  np.array([0.0, -0.6, 0.5]).reshape((3,1)),
                  np.array([0.5, -1.0, 0.5]).reshape((3,1)),
                  np.array([-1.0, 0.0, 0.5]).reshape((3,1))]:
        newton_raphson(xgoal)

if __name__ == "__main__":
    main()
