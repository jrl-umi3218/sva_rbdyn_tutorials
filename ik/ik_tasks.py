import numpy as np

import eigen3 as e
import spacevecalg as sva
import rbdyn as rbd



class BodyTask(object):
  def __init__(self, mb, bodyId, X_O_T, X_b_p=sva.PTransformd.Identity()):
    """
    Compute the error and the jacobian to target a static frame for a body.

    Parameters:
      - mb: MultiBody system
      - bodyId: ID of the body that should move
      - X_0_T: targeted frame (PTransformd)
      - X_b_p: static frame on the body bodyId
    """
    self.bodyIndex = mb.bodyIndexById(bodyId)
    self.X_O_T = X_O_T
    self.X_b_p = X_b_p
    self.jac = rbd.Jacobian(mb, bodyId)
    self.jac_mat_sparse = e.MatrixXd(6, mb.nrDof())

  def X_O_p(self, mbc):
      X_O_b = list(mbc.bodyPosW)[self.bodyIndex]
      return self.X_b_p*X_O_b

  def dimension(self):
    return 6

  def g(self, mb, mbc):
    X_O_p = self.X_O_p(mbc)
    g_body = sva.transformError(self.X_O_T, X_O_p)
    return e.toNumpy(g_body.vector())

  def J(self, mb, mbc):
    X_O_p = self.X_O_p(mbc)
    # set transformation in Origin orientation frame
    X_O_p_O = sva.PTransformd(X_O_p.rotation()).inv()*X_O_p
    jac_mat_dense = self.jac.jacobian(mb, mbc, X_O_p_O)
    self.jac.fullJacobian(mb, jac_mat_dense, self.jac_mat_sparse)
    return e.toNumpy(self.jac_mat_sparse)



class PostureTask(object):
  def __init__(self, mb, q_T):
    """
    Target a default configuration for the robot.

    Parameters:
      - mb: MultiBody system
      - q_T: Targeted configuration (mbc.q size)
    """
    self.q_T = q_T

    def isDefine(j):
        return j.type() in (rbd.Joint.Prism, rbd.Joint.Rev, rbd.Joint.Spherical)
    # take back joint and joint index that are define
    self.jointIndex = [i for i, j in enumerate(mb.joints()) if isDefine(j)]
    self.joints = [mb.joint(index) for index in self.jointIndex]
    nrDof = reduce(lambda dof, j: dof + j.dof(), self.joints, 0)
    self.dim = nrDof

    # initialize g
    self.g_mat = np.mat(np.zeros((nrDof, 1)))

    # initialize the jacobian
    self.J_mat = np.mat(np.zeros((nrDof, mb.nrDof())))
    posInG = 0
    for jIndex, j in zip(self.jointIndex, self.joints):
      posInDof = mb.jointPosInDof(jIndex)
      self.J_mat[posInG:posInG+j.dof(),
                 posInDof:posInDof+j.dof()] = np.eye(j.dof())
      posInG += j.dof()

  def dimension(self):
    return self.dim

  def g(self, mb, mbc):
    q = map(list, mbc.q)
    jointConfig = list(mbc.jointConfig)
    posInG = 0
    for jIndex, j in zip(self.jointIndex, self.joints):
      if j.type() in (rbd.Joint.Prism, rbd.Joint.Rev):
        self.g_mat[posInG:posInG+j.dof(),0] = q[jIndex][0] - self.q_T[jIndex][0]
      elif j.type() in (rbd.Joint.Spherical,):
        orid = e.Quaterniond(*self.q_T[jIndex]).inverse().matrix()
        self.g_mat[posInG:posInG+j.dof(),0] =\
          e.toNumpy(sva.rotationError(orid, jointConfig[jIndex].rotation()))
      posInG += j.dof()
    return self.g_mat

  def J(self, mb, mbc):
    return self.J_mat



class CoMTask(object):
  def __init__(self, mb, com_T):
    """
    Target a fixe CoM position.

    Parameters:
      - mb: MultiBody system
      - com_T: Targeted CoM position (eigen3.Vector3d)
    """
    self.com_T = com_T
    self.comJac = rbd.CoMJacobian(mb)

  def dimension(self):
    return 3

  def g(self, mb, mbc):
    return e.toNumpy(rbd.computeCoM(mb, mbc) - self.com_T)

  def J(self, mb, mbc):
    return e.toNumpy(self.comJac.jacobian(mb, mbc))
