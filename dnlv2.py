from math import sqrt, dist
from dataclasses import dataclass
import numpy

def dot(a, b):
    return a[0]*b[0] + a[1]*b[1]
def vsub(a, b):
    return [a[0] - b[0], a[1] - b[1]]
def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1]]
def vscl(a, b):
    return [a[0] * b, a[1] * b]
def length(a):
    return sqrt(a[0] * a[0] + a[1] * a[1])
def normalize(a):
    ln = length(a)
    if ln==0.0: return [0.0, 0.0]
    return [a[0] / ln, a[1] / ln]

@dataclass
class point:
    origin : list
    velocity : list
    acceleration : list
    force : list
    fix : bool

@dataclass
class line:
    point_a : point
    point_b : point
    length : float
    force : float
    k_spring: float
    k_damp: float

def update(p_list, l_list, dt, gravity, collider_list, point_radius):
    for i in l_list:
        lineDir = normalize( vsub(i.point_b.origin, i.point_a.origin) )#points a to b
        lineDisp = dist(i.point_a.origin, i.point_b.origin)-i.length #signed distance
        damp = dot(vsub(i.point_b.velocity, i.point_a.velocity), lineDir)
        i.force = (lineDisp*i.k_spring + damp*i.k_damp)*(30.0/i.length)
        if not i.point_a.fix:
            i.point_a.acceleration = vadd(i.point_a.acceleration, vscl(lineDir, i.force))
        if not i.point_b.fix:
            i.point_b.acceleration = vadd(i.point_b.acceleration, vscl(lineDir, -i.force))
    
    
    for ptA in range(len(p_list)):
            p_list[ptA].acceleration[1] += gravity

            #check world collision
            
            pdictY = p_list[ptA].origin[1]+p_list[ptA].velocity[1]*dt+p_list[ptA].acceleration[1]*dt*dt
            pdictX = p_list[ptA].origin[0]+p_list[ptA].velocity[0]*dt+p_list[ptA].acceleration[0]*dt*dt
            
            dvY = p_list[ptA].velocity[1]*dt+p_list[ptA].acceleration[1]*dt*dt
            dvX = p_list[ptA].velocity[0]*dt+p_list[ptA].acceleration[0]*dt*dt
            
            reflectX = 0
            reflectY = 0
            friction = 0
            
            for cl in collider_list:
                h0 = (pdictX+point_radius>cl[0]) #left side
                h1 = (pdictX-point_radius<cl[0]+cl[2]) #right side
                v0 = (pdictY+point_radius>cl[1]) #top
                v1 = (pdictY-point_radius<cl[1]+cl[3]) #bottom
                if h0 and h1 and v0 and v1:
                    #h0 intersect
                    if p_list[ptA].origin[0]<cl[0]:
                        p_list[ptA].origin[0] = cl[0]-point_radius
                        reflectX=1
                    #h1 intersect
                    if p_list[ptA].origin[0]>cl[0]+cl[2]:
                        p_list[ptA].origin[0] = cl[0]+cl[2]+point_radius
                        reflectX=1
                    #v0 intersect
                    if p_list[ptA].origin[1]<cl[1]:
                        p_list[ptA].origin[1] = cl[1]-point_radius
                        reflectY=1
                    #v1 intersect
                    if p_list[ptA].origin[1]>cl[1]+cl[3]:
                        p_list[ptA].origin[1] = cl[1]+cl[3]+point_radius
                        reflectY=1
            if (pdictY+point_radius>0):
                p_list[ptA].origin[1] = min(500-point_radius, p_list[ptA].origin[1])
                reflectY=1
            if reflectX:
                p_list[ptA].acceleration[0] *= -0.9
                p_list[ptA].velocity[0] *= -0.9
            if reflectY:
                friction=1
                p_list[ptA].acceleration[1] *= -0.9
                p_list[ptA].velocity[1] *= -0.9
            
            if friction:
                p_list[ptA].acceleration[0]*=0.8
                p_list[ptA].velocity[0]*=0.8
            
            #euler integration
            p_list[ptA].velocity = vadd(p_list[ptA].velocity, vscl(p_list[ptA].acceleration, dt*0.5))
            if p_list[ptA].fix :p_list[ptA].velocity=[0, 0]
            p_list[ptA].origin = vadd(p_list[ptA].origin, vscl(p_list[ptA].velocity, dt))
            p_list[ptA].acceleration = [0, 0]
            
