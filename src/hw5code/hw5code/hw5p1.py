'''hw5p1.py

   This is skeleton code for HW5 Problem 1.  Please EDIT.

   This should simply use NumPy to implement the known forward
   kinematics and Jacobian functions for the 3 DOF robot.

'''

import numpy as np
import math


#
#  EXAMPLE CODE
#
#  TO SEE HOW TO USE NUMPY, LOOK AT NumPyExamples.py!
#
# REMOVE/COMMENT THIS OUT TO AVOID ALL THE EXTRA PRINTS
# import hw5code.NumPyExamples

# def example():
#     print("EXAMPLE CODE:")
    
#     # RANDOM SAMPLE CODE 3x1 column vec, 3x3 matrix:
#     q    = np.array([0.1, 0.3, -0.5]).reshape(-1,1)
#     xdot = np.array([3,   6,    2  ]).reshape(-1,1)
#     J    = Jac(q)
#     qdot = np.linalg.inv(J) @ xdot
#     print("qdot:\n", qdot)

#     print("sin(q):\n",      np.sin(q))
#     print("sin(q[0,0]):\n", np.sin(q[0,0]))
#     print("sin(q[1,0]):\n", np.sin(q[1,0]))
#     print("sin(q[2,0]):\n", np.sin(q[2,0]))
#     # Remember every vector is a 2D object and needs 2 indices!


#
#  Forward Kinematics
#
def fkin(q):
    # EDIT THIS CODE TO DO SOMETHING USEFUL!

    value1 = -np.sin(q[0]) * (np.cos(q[1]) + np.cos(q[1]+q[2]))
    value2 = np.cos(q[0]) * (np.cos(q[1]) + np.cos(q[1]+q[2]))
    value3 = np.sin(q[1]) + np.sin(q[1]+q[2])
    # print("Put the forward kinematics here")
    x = np.array([value1, value2, value3]).reshape(-1,1)

    # Return the tip position as a numpy 3x1 column vector.
    return x


#
#  Jacobian
#
def Jac(q):
    # EDIT THIS CODE TO DO SOMETHING USEFUL!
    # print("Put the Jacobian here")
    # Compute the values
    value1 = -np.cos(q[0]) * (np.cos(q[1]) + np.cos(q[1]+q[2]))
    value2 = np.sin(q[0]) * (np.sin(q[1]) + np.sin(q[1]+q[2]))
    value3 = np.sin(q[0]) * np.sin(q[1]+q[2])

    value4 = -np.sin(q[0]) * (np.cos(q[1]) + np.cos(q[1]+q[2]))
    value5 = -np.cos(q[0]) * (np.sin(q[1]) + np.sin(q[1]+q[2]))
    value6 = -np.cos(q[0]) * np.sin(q[1]+q[2])

    value7 = 0
    value8 = np.cos(q[1]) + np.cos(q[1]+q[2])
    value9 = np.cos(q[1]+q[2])
    J = np.eye(3)
    J = np.random.rand(3,3)
    J = np.array([[value1, value2, value3],
                  [value4, value5, value6],
                  [value7, value8, value9]])

    # Return the Jacobian as a numpy 3x3 matrix.
    return J

def convert_to_rad(q):
    q = [math.radians(q[0]), math.radians(q[1]), math.radians(q[2])]
    return q


#
#  Main Code
#
def main():
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)

    # Run the example code.  FEEL FREE TO REMOVE.
    # example()

    # First (given) test case with following joint coordinates.  Make
    # q a column vector by writing it as a list of lists.
    print("TEST CASE #1:")
    q = np.array([[np.radians(20)],
                  [np.radians(40)],
                  [np.radians(-30)]])
    print('q:\n',       q)
    print('fkin(q):\n', fkin(q))
    print('Jac(q):\n',  Jac(q))

    # Second test case with following joint coordinates.  Make
    # q a column vector by explicitly reshaping.
    print("TEST CASE #2")
    q = np.radians(np.array([30, 30, 60])).reshape(3,1)
    print('q:\n',       q)
    print('fkin(q):\n', fkin(q))
    print('Jac(q):\n',  Jac(q))

if __name__ == "__main__":
    main()
