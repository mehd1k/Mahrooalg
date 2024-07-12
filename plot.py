

import numpy as np
import matplotlib.pyplot as plt
import math
# from shapely.geometry import Point, Polygon
from scipy.spatial import Delaunay
import cvxpy as cp
import gurobipy as gp
from gurobipy import GRB
import mosek
import sys
from scipy.spatial import Delaunay
import  scipy
v0 = [-2.608,-0.422]
v1 = [-2.886,-1.795]
v2 = [-2.784,-3.413]
v3 = [-0.022,-3.559]
v4 = [2.043,-3.208]
v5 = [2.347,-1.525]
v6 = [2.243,0.206]
v7 = [0.019,0.385]

v_ls = np.array([v0, v1, v2, v3, v4, v5, v6, v7])
center = np.mean(v_ls, axis=0)
fig, ax = plt.subplots()  
for i in range(len(v_ls)-1):
    ax.plot([v_ls[i,0], v_ls[i+1,0]], [v_ls[i,1], v_ls[i+1,1]], color = 'black')
        
ax.plot([v_ls[0,0], v_ls[-1,0]], [v_ls[0,1], v_ls[-1,1]], color = 'black')

ax.plot([center[0], v_ls[5,0]], [center[1],v_ls[5,1]], color = 'blue')
ax.plot([center[0], v_ls[7,0]], [center[1],v_ls[7,1]], color = 'blue')
ax.plot([center[0], v_ls[1,0]], [center[1],v_ls[1,1]], color = 'blue')
ax.plot([center[0], v_ls[3,0]], [center[1],v_ls[3,1]], color = 'blue')
# ax.scatter(center[0], center[1])
#########Inner Obstacle
sc = 1.2
o1 = center+ 1*sc*(v_ls[1]-center)/np.linalg.norm((v_ls[1]-center))
o2 = center+ 0.7*sc*(v_ls[3]-center)/np.linalg.norm((v_ls[3]-center))
o3 = center+ 1*sc*(v_ls[5]-center)/np.linalg.norm((v_ls[5]-center))
o4 = center+ 0.7*sc*(v_ls[7]-center)/np.linalg.norm((v_ls[7]-center))
O_ls = np.array([o1, o2, o3, o4])
for i in range(len(O_ls)-1):
    ax.plot([O_ls[i,0], O_ls[i+1,0]], [O_ls[i,1], O_ls[i+1,1]], color = 'red')

ax.plot([O_ls[0,0], O_ls[-1,0]], [O_ls[0,1], O_ls[-1,1]], color = 'red')
    

plt.axis('equal')
plt.show()
