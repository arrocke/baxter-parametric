from geometry_msgs.msg import (
  Vector3,
  Point
)
from math import sqrt

def mag(u):
  return sqrt(u.x ** 2 + u.y ** 2 + u.z ** 2)

def unit(u):
  l = mag(u)
  return Vector3(u.x / l, u.y / l, u.z / l)

def cross(u, v):
  return Vector3(
    u.y * v.z - u.z * v.y,
    u.z * v.x - u.x * v.z,
    u.x * v.y - u.y * v.x
  )

def dot(u, v):
  return u.x * v.x + u.y * v.y + u.z * v.z

def vector(p1, p2):
  return Vector3(
    p2.x - p1.x,
    p2.y - p1.y,
    p2.z - p1.z
  ) 

def scale(s, v):
  return Vector3(
    s * v.x,
    s * v.y,
    s * v.z,
  )

def add(p, v):
  return Point(
    p.x + v.x,
    p.y + v.y,
    p.z + v.z
  )