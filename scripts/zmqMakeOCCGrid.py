#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Occupancy grid creation using a Pioneer pd3x with ultrasonic sensors.

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2023)
"""


import numpy as np
import time
import math as m
import random
import sys
import matplotlib.pyplot as plt
import os
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
#from skimage.draw import line
import cv2

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


client = RemoteAPIClient()
sim = client.getObject('sim')

motorL=sim.getObject('/PioneerP3DX/leftMotor')
motorR=sim.getObject('/PioneerP3DX/rightMotor')
robot = sim.getObject('/PioneerP3DX')

sim.startSimulation()

# Assigning handles to the ultrasonic sensors
usensor = []
Rs = []
Vs = []
for i in range(0,16):
    s = sim.getObject('/PioneerP3DX/ultrasonicSensor['+str(i)+']')
    usensor.append(s)
    q = sim.getObjectQuaternion(s, robot)
    Rs.append(q2R(q[0], q[1], q[2], q[3]))
    Vs.append(np.reshape(sim.getObjectPosition(s, robot), (3,1)))

carpos = sim.getObjectPosition(robot, -1)
carrot = sim.getObjectOrientation(robot, -1)

Kv = 0.5
Kh = 2.5
xd = 3
yd = 3
hd = 0
r = 0.1
L = 0.2
errp = 10

if os.path.exists('map.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('map.txt')
    tocc = 1.0*(occgrid > 0.5)
    occgrid[occgrid > 0.5] = 0
else:
    print('Creating new map')
    occgrid = 0.5*np.ones((100,100))
    tocc = np.zeros((100,100))
t = time.time()

initt = t
niter = 0
while time.time()-t < 30:
    carpos = sim.getObjectPosition(robot, -1)

    xw = carpos[0]
    yw = carpos[1]
    xr = 50 + m.ceil(xw/0.1)
    yr = 50 - m.floor(yw/0.1)
    if xr >= 100:
        xr = 100
    if yr >= 100:
        yr = 100
    occgrid[yr-1, xr-1] = 0

    carrot = sim.getObjectQuaternion(robot, -1)

    uread = []
    ustate = []
    upt = []
    etime = []
    for i in range(0,16,2):
        
        state, distance, point, detectedObj, _ = sim.readProximitySensor(usensor[i])
        
        uread.append(distance)
        upt.append(point)
        ustate.append(state)
        
        # Transform detection from sensor frame to robot frame
        if state == True:
            opos = np.array(point).reshape((3,1))
        else:
            opos = np.array([0,0,1]).reshape((3,1))

        robs = np.matmul(Rs[i], opos) + Vs[i]
        
        # Transform detection from robot frame to global frame
        
        R = q2R(carrot[0], carrot[1], carrot[2], carrot[3])
        rpos = np.array(carpos).reshape((3,1))
        pobs = np.matmul(R, robs) + rpos

        # Transform detection from global frame to occupancy grid cells
        xs = pobs[0]
        ys = pobs[1]
        xo = 50 + m.ceil(xs/0.1)
        yo = 50 - m.floor(ys/0.1)
        if xo >= 100:
            xo = 100
        if yo >= 100:
            yo = 100
        if state:
            tocc[yo-1, xo-1] = 1
        occgrid = cv2.line(occgrid, (xr-1, yr-1), (xo-1, yo-1), (0,0,0), 1)

    # Reactive navigation block
    ul = 1
    ur = 1
    lgains = np.linspace(0,-1,len(upt)//2)
    rgains = np.linspace(-1,0,len(upt)//2)
    for k in range(len(upt)//2):
        if ustate[k]:
            ul = ul + lgains[k]*(1.0 - uread[k])
            ur = ur + rgains[k]*(1.0 - uread[k])
    print('lvel {}   rvel {}'.format(ul, ur))

    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

    niter = niter + 1

print(lgains)
print(rgains)
finalt = time.time()
print('Avg time per iteration ', (finalt-initt)/niter)

sim.setJointTargetVelocity(motorL, 0)
sim.setJointTargetVelocity(motorR, 0)
    
sim.stopSimulation()

plt.imshow(tocc+occgrid)
plt.show()
np.savetxt('map.txt', tocc+occgrid)