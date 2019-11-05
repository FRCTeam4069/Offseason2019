import control as cnt
import math
import numpy as np
import frccontrol as frccnt


def __make_cost_matrix(rho, elems):
    """Creates a cost matrix from the given vector for use with LQR.
    The cost matrix is constructed using Bryson's rule. The inverse square
    of each element in the input is taken and placed on the cost matrix
    diagonal.
    Keyword arguments:
    rho   -- A scaling factor. Large values of rho penalize state excursion,
             small values penalize control effort.
    elems -- a vector. For a Q matrix, its elements are the maximum allowed
             excursions of the states from the reference. For an R matrix,
             its elements are the maximum allowed excursions of the control
             inputs from no actuation.
    Returns:
    State excursion or control effort cost matrix
    """
    return np.diag(rho / np.square(elems))


ROBOT_MAX_VEL = 3.44  # m/s
MAX_EXCURSION_X = 0.3  # m
MAX_EXCURSION_Y = 0.5  # m
MAX_EXCURSION_THETA = math.radians(20)

# LQR constants
Q = __make_cost_matrix(1.0, [MAX_EXCURSION_X, MAX_EXCURSION_Y, MAX_EXCURSION_THETA])
R = __make_cost_matrix(1.0, [ROBOT_MAX_VEL, math.radians(180)])


def A(n):
    return np.array([[0, 0, 0], [0, 0, n], [0, 0, 0]])


B = np.array([[1, 0], [0, 0], [0, 1]])
A1 = A(1e-7)
A2 = A(1)

states = 3
inputs = 2
outputs = 1

C = np.zeros((outputs, states))
D = np.zeros((outputs, inputs))

sys1 = cnt.StateSpace(A1, B, C, D)
sys2 = cnt.StateSpace(A2, B, C, D)
sys1d = sys1.sample(1.0 / 50.0)
sys2d = sys2.sample(1.0 / 50.0)

# python-control gives LQR gains that are far too high for the inputs
# Probably a bug in SLICOT, so just use frccontrol for the calculations
K1 = frccnt.lqr(sys1d, Q, R)
K2 = frccnt.lqr(sys2d, Q, R)

print(f"kx={K1[0, 0]}, ky0={K1[1, 1]}")
print(f"ky1={K2[1, 1]}, ktheta={K2[1, 2]}")
