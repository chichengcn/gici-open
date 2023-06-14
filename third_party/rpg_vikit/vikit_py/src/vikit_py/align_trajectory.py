#!/usr/bin/python

import numpy as np
import vikit_py.transformations as transformations

def align_sim3(model, data):
  """Implementation of the paper: S. Umeyama, Least-Squares Estimation
  of Transformation Parameters Between Two Point Patterns,
  IEEE Trans. Pattern Anal. Mach. Intell., vol. 13, no. 4, 1991.

  Input:
  model -- first trajectory (3xn)
  data -- second trajectory (3xn)

  Output:
  s -- scale factor (scalar)
  R -- rotation matrix (3x3)
  t -- translation vector (3x1)
  t_error -- translational error per point (1xn)

  """

  # substract mean
  mu_M = model.mean(0).reshape(model.shape[0],1)
  mu_D = data.mean(0).reshape(data.shape[0],1)
  model_zerocentered = model - mu_M
  data_zerocentered = data - mu_D
  n = np.shape(model)[0]

  # correlation
  C = 1.0/n*np.dot(model_zerocentered.transpose(), data_zerocentered)
  sigma2 = 1.0/n*np.multiply(data_zerocentered,data_zerocentered).sum()
  U_svd,D_svd,V_svd = np.linalg.linalg.svd(C)
  D_svd = np.diag(D_svd)
  V_svd = np.transpose(V_svd)
  S = np.eye(3)

  if(np.linalg.det(U_svd)*np.linalg.det(V_svd) < 0):
    S[2,2] = -1

  R = np.dot(U_svd, np.dot(S, np.transpose(V_svd)))
  s = 1.0/sigma2*np.trace(np.dot(D_svd, S))
  t = mu_M-s*np.dot(R,mu_D)

  # TODO:
  # model_aligned = s * R * model + t
  # alignment_error = model_aligned - data
  # t_error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),0)).A[0]

  return s, R, t #, t_error

def align_se3(model,data, precision = False):
    """Align two trajectories using the method of Horn (closed-form).

    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)

    Output:
    R -- rotation matrix (3x3)
    t -- translation vector (3x1)
    t_error -- translational error per point (1xn)

    """
    if not precision:
        np.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1).reshape(model.shape[0],1)
    data_zerocentered = data - data.mean(1).reshape(data.shape[0],1)

    W = np.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = np.linalg.linalg.svd(W.transpose())
    S = np.matrix(np.identity( 3 ))
    if(np.linalg.det(U) * np.linalg.det(Vh)<0):
        S[2,2] = -1
    R = U*S*Vh
    t = data.mean(1).reshape(data.shape[0],1) - R * model.mean(1).reshape(model.shape[0],1)

    model_aligned = R * model + t
    alignment_error = model_aligned - data
    t_error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),0)).A[0]

    return R, t, t_error

def _matrix_log(A):
  theta = np.arccos((np.trace(A)-1.0)/2.0)
  log_theta = 0.5*theta/np.sin(theta) * (A - A.transpose())
  x = np.array([log_theta[2,1], log_theta[0,2], log_theta[1,0]])
  return x

def hand_eye_calib(q_gt, q_es, p_gt, p_es, I, delta=10, verbose=True):
  """Implementation of the least squares solution described in the paper:
  Robot Sensor Calibration: Solving AX=XB on the Euclidean Group
  by Frank C. Park and Bryan J. Martin
  """
  n = np.shape(I)[0]
  M = np.zeros([3,3])
  C = np.zeros([3*n, 3])
  b_A = np.zeros([3*n,1])
  b_B = np.zeros([3*n,1])
  for ix, i in enumerate(I):
    A1 = transformations.quaternion_matrix(q_es[i,:])[:3,:3]
    A2 = transformations.quaternion_matrix(q_es[i+delta,:])[:3,:3]
    A  = np.dot(A1.transpose(), A2)
    B1 = transformations.quaternion_matrix(q_gt[i,:])[:3,:3]
    B2 = transformations.quaternion_matrix(q_gt[i+delta,:])[:3,:3]
    B  = np.dot(B1.transpose(), B2)
    alpha = _matrix_log(A)
    beta = _matrix_log(B)
    M = M + np.dot(np.matrix(beta).transpose(), np.matrix(alpha))
    C[3*ix:3*ix+3,:] = np.eye(3) - A
    b_A[3*ix:3*ix+3,0] = np.dot(np.transpose(A1), p_es[i+delta,:]-p_es[i,:])
    b_B[3*ix:3*ix+3,0] = np.dot(np.transpose(B1), p_gt[i+delta,:]-p_gt[i,:])

  # compute rotation
  D,V = np.linalg.linalg.eig(np.dot(M.transpose(), M))
  Lambda = np.diag([np.sqrt(1.0/D[0]), np.sqrt(1.0/D[1]), np.sqrt(1.0/D[2])])
  Vinv = np.linalg.linalg.inv(V)
  X = np.dot(V, np.dot(Lambda, np.dot(Vinv, M.transpose())))

  # compute translation
  d = np.zeros([3*n,1])
  for i in range(n):
    d[3*i:3*i+3,:] = b_A[3*i:3*i+3,:] - np.dot(X, b_B[3*i:3*i+3,:])

  b = np.dot(np.linalg.inv(np.dot(np.transpose(C),C)),  np.dot(np.transpose(C),d))

  return np.array(X),b
