import eigen3 as e
import spacevecalg as sva
import rbdyn as rbd


def TutorialTree():
  """
  Return the MultiBodyGraph, MultiBody and the zeroed MultiBodyConfig with the
  following tree structure:

                b4
             j3 | Spherical
  Root     j0   |   j1     j2     j4
  ---- b0 ---- b1 ---- b2 ----b3 ----b5
  Fixed    RevX   RevY    RevZ   PrismZ
  """

  mbg = rbd.MultiBodyGraph()

  mass = 1.
  I = e.Matrix3d.Identity()
  h = e.Vector3d.Zero()

  rbi = sva.RBInertia(mass, h, I)

  b0 = rbd.Body(rbi, 0, "b0")
  b1 = rbd.Body(rbi, 1, "b1")
  b2 = rbd.Body(rbi, 2, "b2")
  b3 = rbd.Body(rbi, 3, "b3")
  b4 = rbd.Body(rbi, 4, "b4")
  b5 = rbd.Body(rbi, 5, "b5")

  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)
  mbg.addBody(b3)
  mbg.addBody(b4)
  mbg.addBody(b5)

  j0 = rbd.Joint(rbd.Joint.Rev, e.Vector3d.UnitX(), True, 0, "j0")
  j1 = rbd.Joint(rbd.Joint.Rev, e.Vector3d.UnitY(), True, 1, "j1")
  j2 = rbd.Joint(rbd.Joint.Rev, e.Vector3d.UnitZ(), True, 2, "j2")
  j3 = rbd.Joint(rbd.Joint.Spherical, True, 3, "j3")
  j4 = rbd.Joint(rbd.Joint.Prism, e.Vector3d.UnitY(), True, 4, "j4")

  mbg.addJoint(j0)
  mbg.addJoint(j1)
  mbg.addJoint(j2)
  mbg.addJoint(j3)
  mbg.addJoint(j4)

  to = sva.PTransformd(e.Vector3d(0., 0.5, 0.))
  fro = sva.PTransformd.Identity()

  mbg.linkBodies(0, to, 1, fro, 0)
  mbg.linkBodies(1, to, 2, fro, 1)
  mbg.linkBodies(2, to, 3, fro, 2)
  mbg.linkBodies(1, sva.PTransformd(e.Vector3d(0.5, 0., 0.)),
                 4, fro, 3)
  mbg.linkBodies(3, to, 5, fro, 4)

  mb = mbg.makeMultiBody(0, True)
  mbc = rbd.MultiBodyConfig(mb)
  mbc.zero(mb)

  return mbg, mb, mbc
