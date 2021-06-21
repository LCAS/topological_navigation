#!/usr/bin/env python
###################################################################################################################
from __future__ import division
import numpy as np


def pnt2line(pnt, start, end):
    
    pnt = (pnt[:, 0], pnt[:, 1], pnt[:, 2])
    start = (start[:, 0], start[:, 1], start[:, 2])
    end = (end[:, 0], end[:, 1], end[:, 2])
    
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_unitvec = unit(line_vec)
    line_len = length(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    
    t = dot(line_unitvec, pnt_vec_scaled)   
    t[np.where(t < 0.0)[0]] = 0.0
    t[np.where(t > 1.0)[0]] = 1.0
    
    nearest = scale(line_vec, t)
    return distance(nearest, pnt_vec)


def dot(v, w):
    return v[0]*w[0] + v[1]*w[1] + v[2]*w[2]


def length(v):
    return np.sqrt(v[0]**2 + v[1]**2 + v[2]**2)


def vector(b, e):
    return b[0]-e[0], b[1]-e[1], b[2]-e[2]


def unit(v):
    mag = length(v)
    return v[0]/mag, v[1]/mag, v[2]/mag


def distance(p0, p1):
    return length(vector(p0, p1))


def scale(v, sc):
    return v[0]*sc, v[1]*sc, v[2]*sc
###################################################################################################################


###################################################################################################################
# pnt =   [[x0, y0, z0],
#          [x1, y1, z1],
#          [..., ..., ...],
#          [xn, yn, zn]]    

pnt = np.array([[5, 4, 0],
                [5, 4, 0]])
    
start = np.array([[3, 0, 0],
                  [2, 3, 0]])
    
end =   np.array([[8, 5, 0],
                  [4, 7, 0]])
    
print(pnt2line(pnt, start, end))
###################################################################################################################