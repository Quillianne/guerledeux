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




#available at https://www.ensta-bretagne.fr/jaulin/roblib.py
# For help : https://www.ensta-bretagne.fr/jaulin/python.html  
# used in KalMOOC :  https://www.ensta-bretagne.fr/kalmooc/
# used in RobMOOC :  https://www.ensta-bretagne.fr/robmooc/
# used in KalMOOC :  https://www.ensta-bretagne.fr/inmooc/


import numpy as np

from numpy import cos,sin,arctan2,arcsin,array,eye,sum


from numpy.linalg import norm


def scalarprod(u,v): # scalar product
    u,v=u.flatten(),v.flatten()
    return sum(u[:]*v[:])




def eulermat2angles(R):
    φ=arctan2(R[2,1],R[2,2])
    θ=-arcsin(R[2,0])
    ψ=arctan2(R[1,0],R[0,0])
    return φ,θ,ψ


def eulerderivative(φ,θ,ψ):
    cφ,sφ,cθ,sθ,tθ,cψ,sψ = cos(φ),sin(φ),cos(θ),sin(θ),sin(θ)/cos(θ),cos(ψ),sin(ψ)        
    return array([[1,sφ*tθ,cφ*tθ],[0, cφ,-sφ],[0,sφ/cθ,cφ/cθ]])    
    
def angle(x):
    x=x.flatten()
    return arctan2(x[1],x[0])


def angle2d(u,v):
    u1,u2=u[0,0],u[1,0]
    v1,v2=v[0,0],v[1,0]
    dx=u1*v1+u2*v2
    dy=u1*v2-u2*v1
    return arctan2(dy,dx)



def rotuv(u,v): #returns rotation with minimal angle  such that  v=R*u
            # see https://en.wikipedia.org/wiki/Rotation_matrix#Vector_to_vector_formulation
    u=array(u).reshape(3,1)
    v=array(v).reshape(3,1)
    u=(1/norm(u))*u
    v=(1/norm(v))*v
    c=scalarprod(u,v)
    A=v@u.T-u@v.T
    return eye(3,3)+A+(1/(1+c))*A@A

def adjoint(w):
    if isinstance(w, (float, int)): return array([[0,-w] , [w,0]])
    #print('w=',w)
    w=tolist(w)
    #print('tolist(w)=',w)
    return array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

def tolist(w):
    if isinstance(w,(list)): return w
    return list(w.flatten())
