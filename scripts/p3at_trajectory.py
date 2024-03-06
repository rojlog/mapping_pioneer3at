"""
Trajectory following for a Pioneer P3DX robot

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2024)
"""

import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

import math as m

ttime = 80
xarr = np.array([-2,-1,0,0,0,0,0,1,2])
yarr = np.array([-2,-2,-2,-1,0,1,2,2,2])
tarr = np.linspace(0, ttime, xarr.shape[0])

xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)

def q2R(x,y,z,w):
    R = np.zeros((3,3))
    R[0,0] = 1-2*(y**2+z**2)
    R[0,1] = 2*(x*y-z*w)
    R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w)
    R[1,1] = 1-2*(x**2+z**2)
    R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w)
    R[2,1] = 2*(y*z+x*w)
    R[2,2] = 1-2*(x**2+y**2)
    return R


def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

print('Program started')



# Assigning handles to the ultrasonic sensors
usensor = []
Rs = []
Vs = []
for i in range(0,16):
    s = sim.getObject('/PioneerP3DX/ultrasonicSensor['+str(i)+']')
    usensor.append(s)
    q = sim.getObjectQuaternion(s, robot)
    Rs.append(q2R(q[0], q[1], q[2], q[3]))
    Vs.append(sim.getObjectPosition(s, robot))

sim.startSimulation()


Kv = 0.5
Kh = 1.2
r = 0.5*0.195
L = 0.311

errp = 1000

xobs = []
yobs = []
xrob = []
yrob = []

tstart = sim.getSimulationTime()
while sim.getSimulationTime()-tstart < ttime:
    #while errp > 0.01:
    ctime = sim.getSimulationTime()-tstart
    xd = spi.splev(ctime, xc, der=0)
    yd = spi.splev(ctime, yc, der=0)
    carpos = sim.getObjectPosition(robot, -1)
    Vr = np.array(carpos)
    xrob.append(carpos[0])
    yrob.append(carpos[1])
    carrot = sim.getObjectOrientation(robot, -1)
    theta = carrot[2]
    Rr = np.array([[np.cos(theta), -np.sin(theta), 0],
                   [np.sin(theta),  np.cos(theta), 0],
                   [0, 0, 1]])
    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
    angd = m.atan2(yd-carpos[1], xd-carpos[0])
    errh = angdiff(carrot[2], angd)
    print('Distance to goal: {}   Heading error: {}'.format(errp, errh))

    v = Kv*errp
    omega = Kh*errh

    ur, ul = v2u(v, omega, r, L)
    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

    for k in range(16):
        result, distance, _, _, _ = sim.readProximitySensor(usensor[k])
        if result > 0:
            opoint = np.array([0,0,distance])
            opoint = Rs[k] @ opoint + Vs[k]
            opoint = Rr @ opoint + Vr
            xobs.append(opoint[0])
            yobs.append(opoint[1])

sim.setJointTargetVelocity(motorL, 0.0)
sim.setJointTargetVelocity(motorR, 0.0)

plt.plot(xrob, yrob)
plt.plot(xobs, yobs, '.')
plt.show()

sim.stopSimulation()