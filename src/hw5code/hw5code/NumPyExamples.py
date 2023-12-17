'''NumPyExamples.py

   This is sample code for using NumPy.

'''

import numpy as np


# Welcome.
print("Hello - this is NumPy example code")


# 1D ARRAYS = BAD
#
# The basic NumPy element is an array.  But CAUTION, NumPy wants to
# make this a 1-dimensional list.  This is NOT what we want.
badarray = np.array([1, 2, 3, 4])
print(badarray, "is a 1D array which is BAD - shape is", badarray.shape)
print()


# COLUMN VECTORS, ROW VECTORS, MATRICES are 2D ARRAYS (ARRAYS of ARRAYS)
#
# All our vectors (whether column or row) and matrices have to be 2D
# arrays so we can properly multiply things.  They To construct,
# EITHER give a list of lists.  OR reshape a 1D array into the right
# size!  When reshaping, "-1" means use whatever length is needed.
rowvec1 = np.array([[1, 2, 3, 4]])
rowvec2 = np.array([ 5,  6,  7,  8]).reshape(1, 4)
rowvec3 = np.array([ 9, 10, 11, 12]).reshape(1, -1)
print(rowvec1, "is a 2D array = row vector with shape", rowvec1.shape)
print(rowvec2, "is a 2D array = row vector with shape", rowvec2.shape)
print(rowvec3, "is a 2D array = row vector with shape", rowvec3.shape)
print()

colvec1 = np.array([[1], [2], [3]])
colvec2 = np.array([4, 5, 6]).reshape(3, 1)
colvec3 = np.array([7, 8, 9]).reshape(-1, 1)
print(colvec1, "is a 2D array = col vector with shape", colvec1.shape)
print(colvec2, "is a 2D array = col vector with shape", colvec2.shape)
print(colvec3, "is a 2D array = col vector with shape", colvec3.shape)
print()

matrix1 = np.array([[1, 2, 3],
                    [4, 5, 6],
                    [7, 8, 9]])
print(matrix1, "is a 2D array = 3x3 matrix with shape", matrix1.shape)
print()


# ADDITION
#
# Works as expected :)
print("ADDITION")
print(colvec2 + colvec1)
print(colvec2 - colvec1)
print()


# MULTIPLYING ***CAREFUL***
#
# NumPy multiplication returns element-by-element products.  So TO
# MULTIPLY "mathematical" matrix products (or dot products), use
# either special functions or the @ operator.
row = np.array([-2, -1, 0]).reshape(1, -1)
col = np.array([1, 0.5, -1]).reshape(-1, 1)
mat = np.array([[1, 2, 3],
                [2, 2, 2],
                [1, 0, 1]])

a = mat @ col
b = row @ col

print("MULTIPLICATION")
print("row: \n", row)
print("col: \n", col)
print("mat: \n", mat)

print("mat @ col: \n", mat @ col)
print("row @ mat: \n", row @ mat)

print("mat @ mat: \n", mat @ mat)
print("col @ row: \n", col @ row)
print("row @ col: \n", row @ col)
print()


# TRANSPOSE AND INVERSE
#
# Multiple ways to do this.  Easiest for transpose just do ".T".
# Inverse is function call.
print("MATRIX")
print(mat)

print("TRANSPOSE OPTIONS")
print(mat.T)
print(mat.transpose())
print(np.transpose(mat))

print("INVERSE")
print(np.linalg.inv(mat))
print(mat @ np.linalg.inv(mat))
print()


# EXTRACT SUB-MATRICES/VECTORS/ELEMENTS
#
# Pulling out elements from matrices/vectors.  CAUTION: Pulling out a
# vector, NumPy tries to create 1D objects, which is BAD.

print("3x3 matrix:\n",        mat)
print("Top left element:\n",  mat[0:1,0:1])
print("Top right 2x2:\n",     mat[0:2,1:3])
print("Middle col vector:\n", mat[:,1:2])

# DO NOT DO   mat[:,1]   -> creates a 1D list!!  BAD!!
# INSTEAD DO  mat[:,1:2] -> same time, but 2D = col vector!!  GOOD!

# Remember you are pulling from a 2d object -> 2 indices!
print("Single element:\n",    mat[1,2])


# ROTATION AND TRANSFORM MATRICES
#
# See TransformHelpers.py


# OTHER FUNCTIONS
#
# Also useful (eventually) might be: Stacking, pseudo inverse, SVD, norms.
print("Vertical Stacking 3x1 vec1 ontop of 3x1 vec2")
sixbyone = np.vstack((colvec1, colvec2))
print(sixbyone)

print("Horizontal stacking left next to right")
threebysix = np.hstack((mat, mat))
print(threebysix)

pseudo = np.linalg.pinv(threebysix)
print("Matrix pseudo inverse\n",  pseudo)

(U, S, VT) = np.linalg.svd(threebysix)
print("Matrix SVD:")
print("U:\n", U)
print("S:\n", S)
print("VT:\n", VT)
k = len(S)
print("U @ S @ VT\n", U[:,0:k] @ np.diag(S) @ VT[0:k,:])

n = np.linalg.norm(sixbyone)
print("Vector norm\n", n)

