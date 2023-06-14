# -*- coding: utf-8 -*-
"""
Created on Wed Aug  7 22:13:06 2013

@author: cforster
"""

import numpy as np

def unproject(a):
  """Makes a vector homogeneous"""
  return np.append(a, 1)

def project(a):
  """De-homogenises a vector"""
  return a[:-1]/float(a[-1])

def skew(v):
  """Returns the skew-symmetric matrix of a vector"""
  return np.matrix([[0, -v[2], v[1]],
                    [v[2], 0, -v[0]],
                    [-v[1], v[0], 0]], dtype=np.float32)