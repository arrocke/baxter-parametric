#!/usr/bin/env python

from draw import ParametricDrawer
from math import (
  pi,
  sin,
  cos
)

d = ParametricDrawer()

# Circle
# def x(t):
#   return cos(t)
# def y(t):
#   return sin(t)
# start = 0
# end = 2 * pi
# steps = 100

# Astroid
def x(t):
  return cos(t) ** 3
def y(t):
  return sin(t) ** 3
start = 0
end = 2 * pi
steps = 100


d.draw(x, y, start, end, steps)
