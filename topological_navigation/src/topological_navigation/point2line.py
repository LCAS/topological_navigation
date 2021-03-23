#!/usr/bin/env python
###################################################################################################################
import math


def pnt2line(pnt, start, end):
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    t = dot(line_unitvec, pnt_vec_scaled)    
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return (dist, nearest)


def dot(v,w):
    x,y,z = v
    X,Y,Z = w
    return x*X + y*Y + z*Z


def length(v):
    x,y,z = v
    return math.sqrt(x*x + y*y + z*z)


def vector(b,e):
    x,y,z = b
    X,Y,Z = e
    return (X-x, Y-y, Z-z)


def unit(v):
    x,y,z = v
    mag = length(v)
    if mag == 0:
        return (0, 0, 0)
    return (x/mag, y/mag, z/mag)


def distance(p0,p1):
    return length(vector(p0,p1))


def scale(v,sc):
    x,y,z = v
    return (x * sc, y * sc, z * sc)


def add(v,w):
    x,y,z = v
    X,Y,Z = w
    return (x+X, y+Y, z+Z)    
###################################################################################################################