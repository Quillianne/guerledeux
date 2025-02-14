# MIT License

# Copyright (c) [2022] [Luc Jaulin]

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.




# Available at https://www.ensta-bretagne.fr/jaulin/roblib.py
# For help: https://www.ensta-bretagne.fr/jaulin/python.html  
# Used in KalMOOC: https://www.ensta-bretagne.fr/kalmooc/
# Used in RobMOOC: https://www.ensta-bretagne.fr/robmooc/
# Used in KalMOOC: https://www.ensta-bretagne.fr/inmooc/

import numpy as np
from numpy import cos, sin, arctan2, arcsin, array, eye, sum
from numpy.linalg import norm

def scalarprod(u, v):
    """Computes the scalar (dot) product of two vectors."""
    u, v = u.flatten(), v.flatten()
    return sum(u * v)

def eulermat2angles(R):
    """
    Converts an Euler rotation matrix to Euler angles.
    
    Parameters
    ----------
    R : array_like
        3x3 rotation matrix.
    
    Returns
    -------
    tuple
        Euler angles (phi, theta, psi) in radians.
    """
    phi = arctan2(R[2, 1], R[2, 2])
    theta = -arcsin(R[2, 0])
    psi = arctan2(R[1, 0], R[0, 0])
    return phi, theta, psi

def eulerderivative(phi, theta, psi):
    """
    Computes the transformation matrix that relates the derivatives of Euler angles
    to the angular velocity.
    
    Parameters
    ----------
    phi, theta, psi : float
        Euler angles in radians.
    
    Returns
    -------
    ndarray
        3x3 transformation matrix.
    """
    cphi, sphi = cos(phi), sin(phi)
    ctheta, stheta = cos(theta), sin(theta)
    ttheta = stheta / ctheta
    cpsi, spsi = cos(psi), sin(psi)
    return array([[1, sphi * ttheta, cphi * ttheta],
                  [0, cphi,         -sphi],
                  [0, sphi / ctheta, cphi / ctheta]])

def angle(x):
    """
    Computes the angle of a 2D vector.
    
    Parameters
    ----------
    x : array_like
        2D vector.
    
    Returns
    -------
    float
        Angle in radians.
    """
    x = x.flatten()
    return arctan2(x[1], x[0])

def angle2d(u, v):
    """
    Computes the signed angle between two 2D vectors.
    
    Parameters
    ----------
    u, v : ndarray
        2D column vectors.
    
    Returns
    -------
    float
        Signed angle in radians.
    """
    u1, u2 = u[0, 0], u[1, 0]
    v1, v2 = v[0, 0], v[1, 0]
    dx = u1 * v1 + u2 * v2
    dy = u1 * v2 - u2 * v1
    return arctan2(dy, dx)

def rotuv(u, v):
    """
    Returns the rotation matrix with the minimal angle such that v = R * u.
    See https://en.wikipedia.org/wiki/Rotation_matrix#Vector_to_vector_formulation
    
    Parameters
    ----------
    u, v : array_like
        3D vectors.
    
    Returns
    -------
    ndarray
        3x3 rotation matrix.
    """
    u = array(u).reshape(3, 1)
    v = array(v).reshape(3, 1)
    u = (1 / norm(u)) * u
    v = (1 / norm(v)) * v
    c = scalarprod(u, v)
    A = v @ u.T - u @ v.T
    return eye(3) + A + (1 / (1 + c)) * (A @ A)

def adjoint(w):
    """
    Returns the adjoint matrix of a vector.
    
    Parameters
    ----------
    w : array_like or float
        A 3D vector or a scalar.
    
    Returns
    -------
    ndarray
        The adjoint matrix.
    """
    if isinstance(w, (float, int)):
        return array([[0, -w], [w, 0]])
    w = tolist(w)
    return array([[0, -w[2], w[1]],
                  [w[2], 0, -w[0]],
                  [-w[1], w[0], 0]])

def tolist(w):
    """
    Converts an array to a list.
    
    Parameters
    ----------
    w : array_like or list
        Input array or list.
    
    Returns
    -------
    list
        The flattened list.
    """
    if isinstance(w, list):
        return w
    return list(w.flatten())