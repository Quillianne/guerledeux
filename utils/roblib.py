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

from numpy import mean,pi,cos,sin,sinc,sqrt,tan,arctan,arctan2,tanh,arcsin,arccos,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,vstack,hstack,diag,median,\
                    sign,sum,meshgrid,cross,linspace,append,round,trace,rint

from numpy.random import randn,rand,uniform
from numpy.linalg import inv, det, norm, eig,qr
from scipy.linalg import sqrtm,expm,logm,norm,block_diag

from scipy.signal import place_poles
from math import factorial




# Unicode https://en.wikipedia.org/wiki/List_of_Unicode_characters
# αβδεθλΛμρτφψωΓ




def scalarprod(u,v): # scalar product
    u,v=u.flatten(),v.flatten()
    return sum(u[:]*v[:])


def eulermat(φ,θ,ψ):
    return expw([0,0,ψ]) @ expw([0,θ,0]) @ expw([φ,0,0])


def eulermat2angles(R):
    φ=arctan2(R[2,1],R[2,2])
    θ=-arcsin(R[2,0])
    ψ=arctan2(R[1,0],R[0,0])
    return φ,θ,ψ


def rot2w(R): return adjoint_inv(logm(R))

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



#def angle3dold(u,v):  #returns θ*w such that  v=expw(θ*w)*u  with θ minimal in [0,pi]
#    u=(1/norm(u))*array(u)
#    v=(1/norm(v))*array(v)
#    c=scalarprod(u,v)
#    w=adjoint(u)@v
#    if c > 0.99: return w
#    if c <-0.99:return angle3d(u,v+0.01*randn(3,1))
#    return (arccos(c)/norm(w))*w

def rotuv(u,v): #returns rotation with minimal angle  such that  v=R*u
            # see https://en.wikipedia.org/wiki/Rotation_matrix#Vector_to_vector_formulation
    u=array(u).reshape(3,1)
    v=array(v).reshape(3,1)
    u=(1/norm(u))*u
    v=(1/norm(v))*v
    c=scalarprod(u,v)
    A=v@u.T-u@v.T
    return eye(3,3)+A+(1/(1+c))*A@A

def angle3d(u,v):  #returns θ*w such that  v=expw(θ*w)*u  with θ minimal in [0,pi]
    u=array(u).reshape(3,1)
    v=array(v).reshape(3,1)
    if norm(u)<0.0001: return angle3d(u+0.01*randn(3,1),v)
    if norm(v)<0.0001: return angle3d(u,v+0.01*randn(3,1))
    u=(1/norm(u))*u
    v=(1/norm(v))*v
    if scalarprod(u,v) <-0.999: return angle3d(u+0.01*randn(3,1),v+0.01*randn(3,1))
    return logw(rotuv(u,v))

    
def add1(M):
    M=array(M)
    return vstack((M,ones(M.shape[1])))

def tolist(w):
    if isinstance(w,(list)): return w
    return list(flatten(w))

def adjoint(w):
    if isinstance(w, (float, int)): return array([[0,-w] , [w,0]])
    #print('w=',w)
    w=tolist(w)
    #print('tolist(w)=',w)
    return array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

def ad(w):
    return adjoint(w)


def adjoint_inv(A):
    if size(A)==4:  return A[1,0]  # A is 2x2
    return array([[A[2,1]],[A[0,2]],[A[1,0]]]) # A is 3x3

def expw(w): return expm(adjoint(w))
def expwH(w): return ToH(expw(w))
def logw(R): return adjoint_inv(logm(R))

def Rlatlong(lx,ly): 
    return eulermat(0,0,lx)@eulermat(0,-pi/2+ly,-pi/2).T
        

def latlong2cart(ρ,lx,ly): 
    return ρ*array([[cos(ly)*cos(lx)],[cos(ly)*sin(lx)],[sin(ly)] ])  

def cart2latlong(x,y,z):
    r=norm(array([x,y,z]))
    ly=arcsin(z/r) 
    lx=arctan2(y,x)
    return (r,lx,ly)            

def tran2H(x,y):
    return array([[1,0,x],[0,1,y],[0,0,1]])

def rot2H(a):
    return array([[cos(a),-sin(a),0],[sin(a),cos(a),0],[0,0,1]])

def arrow2H(L):
    e=0.2
    return add1(L*array([[0,1,1-e,1,1-e],[0,0,-e,0,e]]))

#

def ToH(R,v=array([[0],[0],[0]])):  # transformation matrix to homogenous
    H = hstack((R,v))
    V = vstack((H, array([0,0,0,1])))
    return V

def tran3H(x, y, z):
    return array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])



def rot3H(wx, wy, wz): return ToH(expw([wx,wy,wz]))


def eulerH(φ,θ,ψ):
    return ToH(expw([0,0,ψ]) @ expw([0,θ,0]) @ expw([φ,0,0]))





def cube3H():
    return array([[0,1,1,0,0,0,1,1,0,0,0,0,1,1,1,1],
               [0,0,0,0,0,1,1,1,1,1,1,0,0,1,1,0],
               [0,0,1,1,0,0,0,1,1,0,1,1,1,1,0,0],
               [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])







def auv3H():
    return add1(array([ [0.0,0.0,10.0,0.0,0.0,10.0,0.0,0.0],
                   [-1.0,1.0,0.0,-1.0,-0.2,0.0,0.2,1.0],
                   [0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0]]))


def boat3H():
    b=1.5
    return add1(array([ [-2, 6, 6,-2,-2,-2,   9, 9, -2,-2,-2,-2,-2, 9,9,6, 6,9],
                        [1,  1,-1,-1, 1, b,   b,-b, -b, b, 1,-1,-b,-b,b,1,-1,-b],
                        [0,  0, 0, 0, 0, 1,   1, 1,  1, 1, 0, 0, 1, 1,1,0, 0,1 ]]))


def earth3H(ρ):
    a = pi/10
    Lx = arange(0, 2*pi+a, a)
    Ly = arange(-pi/2, pi/2+a, a)
    M1 = ρ*array([[cos(-pi/2)*cos(0)],[cos(-pi/2)*sin(0)],[sin(-pi/2)]])
    M2=M1
    for ly1 in Ly:
        for lx1 in Lx:
            M1 = hstack((M1, ρ*array([[cos(ly1)*cos(lx1)],[cos(ly1)*sin(lx1)],[sin(ly1)]])))
    for lx1 in Lx:
        for ly1 in Ly:
            M2 = hstack((M2, ρ*array([[cos(ly1)*cos(lx1)],[cos(ly1)*sin(lx1)],[sin(ly1)]])))
    M=hstack((M1, M2))
    return add1(M)


def cylinder3H(r,L):
    n = 25
    W=[[0.3,0],[0,0],[0,0]]
    for i in range(n+1):
        P1=[[0],[r*cos(2*pi*i/n)],[r*sin(2*pi*i/n)]]
        P2=[[L],[r*cos(2*pi*i/n)],[r*sin(2*pi*i/n)]]
        W=hstack((W,P1,P2,P1,[[0],[0],[0]]))
    return add1(array(W))

def tondarray(M):
    if type(M)==float:
        return array([[M]])
    elif type(M)==int:
        return array([[M]])        
    else:
        return M    


def mvnrnd(xbar,Γ,n): 
    X=randn(2,n)
    X = (xbar @ ones((1,n))) + sqrtm(Γ) @ X
    return(X)    



def mvnrnd2(x,G): 
    n=len(x)
    x1=x.reshape(n)
    y = np.random.multivariate_normal(x1,G).reshape(n,1)
    return(y)    

def mvnrnd1(G):
    G=tondarray(G)
    n=len(G)
    x=array([[0]] * n)
    return(mvnrnd2(x,G))  
    

def kalman_predict(xup,Gup,u,Γα,A):
    Γ1 = A @ Gup @ A.T + Γα
    x1 = A @ xup + u    
    return(x1,Γ1)    

def kalman_correc(x0,Γ0,y,Γβ,C):
    S = C @ Γ0 @ C.T + Γβ        
    K = Γ0 @ C.T @ inv(S)           
    ytilde = y - C @ x0        
    Gup = (eye(len(x0))-K @ C) @ Γ0 
    xup = x0 + K@ytilde
    return(xup,Gup) 
    
def kalman(x0,Γ0,u,y,Γα,Γβ,A,C):
    xup,Gup = kalman_correc(x0,Γ0,y,Γβ,C)
    x1,Γ1=kalman_predict(xup,Gup,u,Γα,A)
    return(x1,Γ1)     


def place(A,B,poles):
    return place_poles(A,B,poles).gain_matrix
  
def loadcsv(file1):
    fichier = open(file1,'r')
    D = fichier.read().split("\n")
    fichier.close()
    for i in range(len(D)):
        D[i] = D[i].split(";")
    D = array([[float(elt) for elt in Ligne] for Ligne in D])
    return D



def sawtooth(x):
    return (x+pi)%(2*pi)-pi   # or equivalently   2*arctan(tan(x/2))

def projSO3(M):   # return a rotation matrix close to M
    Q,R = np.linalg.qr(M)
    return Q@diag(diag(sign(R)))


#    A=array([[2,1],[-1,2]])
#    print(logm(A))

#    print('main program to test the functions of roblib.py')
#    u=array([[1],[0],[0]])
#    v=array([[1.1],[0],[0]])
#    w=angle3d(u, v)
#    print('w=',w)

#    R=eulermat(1,2,3)
#    D=diag((1,2,3))
#    A=R@D@R.T
    #ax = figure3D()
    #clean3D(ax, -10, 10, -10, 10,-10, 10)
    #draw_axis3D(ax, 0, 0, 0, 3 * eye(3, 3))
    #draw_robot3D(ax, array([[2],[3],[4]]), eye(3, 3), 'blue', 0.3)
    #pause(1)

        #φ, θ, ψ=-0.1,-0.2,-0.3
    #R=eulermat(φ, θ, ψ)
    #print(R)
    #φ1, θ1, ψ1=eulermat2angles(R)
    #print(φ1, θ1, ψ1)


    #     φ,θ,ψ=list(x[3:6])
    #    s,θ,ds,dθ =list(x[0:4,0])  #select the components of a vector
    # p = x[[0,1,3]] # forms the subvector associated to comppnents 1,2,4

#    print("v=",v)
#    print("R1=",R1)
#    w=R1[:,2] # rotation vector
#    print('R@w=',R@w) #  R*w % we check that it an eigen vector associated to 1 
#
#    α=arccos(0.5*(trace(R)-1))
#    print ("angle associated to w:",α)
#    M = expm(α*adjoint(w))
#    print(M)
#    print(R)
         

    
    
    
#    np.set_printoptions(threshold=np.nan)  # print vectors in the console without "..."
#    R=zeros((3,4))
#    x=[[1],[2],[3]]
#    print('R1=',R1)
#          
#    demo_animation()
#    demo_random()

#    demo_field()
    
#
#    M=array([[1,2],[5,6],[9,10]])
#    print(M)
#    x=array([[1], [2]])    
#    x2= M@x  #multiplication dans Python 3
#    
##
#    G = array([[1, 0], [0, 1]])
#    x3=mvnrnd2(x,G)
#    print("x3=",x3)
#    
#    x4=mvnrnd1(G)
#    print(x4)
#    
#    
#    
