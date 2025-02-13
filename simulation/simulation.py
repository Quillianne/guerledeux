from roblib import *


def f(x, u):
    x  = x.flatten()
    u = u.flatten()
    return array([[x[3]*cos(x[2])],
                  [x[3]*sin(x[2])], 
                  [u[0]], 
                  [u[1]]])


def c(i, t):
    return array([[cos(a*t + 2*i*pi/m)],
                  [sin(a*t + 2*i*pi/m),]])

def dc(i, t):
    return a * array([[-sin(a*t + 2*i*pi/m)],
                      [cos(a*t + 2*i*pi/m)]])

def ddc(i, t):
    return -a**2 * c(i, t)


def control(x, w, dw, ddw):

    v = (w - x[:2]) + 2*(dw - x[3,0]*array([[cos(x[2,0])], [sin(x[2,0])]])) + ddw
    A = array([[-x[3,0]*sin(x[2,0]), cos(x[2,0])],
               [x[3,0]*cos(x[2,0]), sin(x[2,0])]])

    return inv(A) @ v



# parameters
a = 0.1
m = 6
X = 10*rand(m, 4, 1)
dt = 0.1

ax = init_figure(-5, 5, -5, 5)

# simulation
for t in arange(0, 20, dt):
    clear(ax)
    # for each boat
    for i in range(m):
        u = control(X[i], c(i, t), dc(i, t), ddc(i, t))
        plot(c(i, t)[0], c(i, t)[1], 'r+')
        X[i] = X[i] + dt*f(X[i], u)
        draw_tank(X[i][:3], r=0.05)
    pause(0.0001)
show()


