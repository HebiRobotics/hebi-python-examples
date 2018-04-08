import numpy as np
from math import cos, sin


def rotate_x(angle, dtype=np.float64):
  c = cos(angle)
  s = sin(angle)
  return np.matrix([
    [1.0, 0.0, 0.0],
    [0.0, c, -s],
    [0.0, s, c]
    ], dtype=np.float64)


def rotate_y(angle, dtype=np.float64):
  c = cos(angle)
  s = sin(angle)
  return np.matrix([
    [c, 0.0, s],
    [0.0, 1.0, 0.0],
    [-s, 0.0, c]
    ], dtype=np.float64)


def rotate_z(angle, dtype=np.float64):
  c = cos(angle)
  s = sin(angle)
  return np.matrix([
    [c, -s, 0.0],
    [s, c, 0.0],
    [0.0, 0.0, 1.0]
    ], dtype=np.float64)

