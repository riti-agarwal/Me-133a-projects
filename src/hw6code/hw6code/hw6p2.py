import numpy as np
import math
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


x_r_dot = np.array([1,0,1]).reshape(3,1)

def main():
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)

    #part a 
    q = np.radians(np.array([0.0,44.5,90.0]))
    Jaqq = Jac(q)
    print("Part a: J(q) is ", Jaqq)
    U, s_i, Vt = np.linalg.svd(Jaqq)
    print("U is: ", U)
    print("diag s is: ", np.diag(s_i))
    print("V transpose is: ", Vt)

    #part b
    Jinv = np.linalg.inv(Jaqq)
    q_dot_b = Jinv @ x_r_dot
    x_dot_b = Jaqq @ q_dot_b
    print("Part b: ")
    print("q_dot is: ", q_dot_b)
    print('x_dot is: ', x_dot_b)

    #part c
    M = 3
    # for part d, gamma = 0.1
    gamma = 0.01
    Jt = np.transpose(Jaqq)
    Im = np.eye(M)
    Jwinv = Jt @ (np.linalg.inv((Jaqq @ Jt) + (gamma**2 * Im)))
    q_dot_c = Jwinv @ x_r_dot
    x_dot_c = Jaqq @ q_dot_c
    print("Part c: ")
    print("q_dot is: ", q_dot_c)
    print('x_dot is: ', x_dot_c)

    #part e
    gamma_e = 0.1
    s_inv = np.zeros(s_i.shape)
    for i in range(len(s_i)):
        if s_i[i] >= gamma_e:
            s_inv[i] = 1 / s_i[i]
        else:
            s_inv[i] = s_i[i]/ (gamma_e**2)
    q_dot_e = (np.transpose(Vt) @ np.diag(s_inv) @ np.transpose(U)) @ x_r_dot
    x_dot_e = Jaqq @ q_dot_e
    print("Part e: ")
    print("q_dot is: ", q_dot_e)
    print('x_dot is: ', x_dot_e)



if __name__ == "__main__":
    main()
