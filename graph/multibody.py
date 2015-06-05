import rbdyn as rbd

from body import linesBody, meshBody, endEffectorBody
from joint import revoluteJoint, prismaticJoint, sphericalJoint
from transform import setActorTransform



JOINT_FROM_TYPE = {
  rbd.Joint.Rev: revoluteJoint,
  rbd.Joint.Prism: prismaticJoint,
  rbd.Joint.Spherical: sphericalJoint
}


class MultiBodyViz(object):
  def __init__(self, mbg, mb, meshDict={}, endEffectorDict={}):
    """
    MutiBody visualisation.
    Params:
      - meshDict : {bodyName: fileName}
      - endEffectorDict : {bodyName, X_to_end_effector, size, color}
    """
    self.aBodies = [] # (body index, actor, X_s)
    self.aJoints = [] # (joint index, actor, X_s)

    successorJointsId = dict(mbg.successorJoints(mb.body(0).id()))

    # body displayed by a line
    lineBodiesByIndex = {bi:b for bi, b in enumerate(mb.bodies()) if
                         len(list(successorJointsId[b.id()])) > 0}

    # create actor from mesh
    for bodyName, fileName in meshDict.items():
      bodyId = mbg.bodyIdByName(bodyName)
      bi = mb.bodyIndexById(bodyId)
      del lineBodiesByIndex[bi] # don't create a line body if a mesh is set
      a, X_s = meshBody(fileName)
      self.aBodies.append((bi, a, X_s))

    # create actor from end effector
    for bodyName, (X_see, size, color) in endEffectorDict.items():
      bodyId = mbg.bodyIdByName(bodyName)
      bi = mb.bodyIndexById(bodyId)
      a, X_s = endEffectorBody(X_see, size, color)
      self.aBodies.append((bi, a, X_s))

    for bi, b in lineBodiesByIndex.items():
      a, X_s = linesBody(mb, b.id(), successorJointsId)
      self.aBodies.append((bi, a, X_s))

    for ji, j in enumerate(mb.joints()):
      try:
        a, X_s = JOINT_FROM_TYPE[j.type()](j)
        self.aJoints.append((ji, a, X_s))
      except KeyError:
        pass


  def display(self, mb, mbc):
    """
    Display the MultiBody.
    """
    bodyPosW = list(mbc.bodyPosW)

    for bi, actor, X_s in self.aBodies:
      X_a = X_s*bodyPosW[bi]
      setActorTransform(actor, X_a)

    for ji, actor, X_s in self.aJoints:
      X_a = X_s*bodyPosW[ji]
      setActorTransform(actor, X_a)


  def addActors(self, scene):
    """
    Add all actors to the scene.
    """
    actors = map(lambda ab: ab[1], self.aBodies) +\
      map(lambda aj: aj[1], self.aJoints)
    scene.add_actors(actors)



if __name__ == '__main__':
  import numpy as np
  import eigen3 as e
  import spacevecalg as sva

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

  #                b4
  #             j3 | Spherical
  #  Root     j0   |   j1     j2     j4
  #  ---- b0 ---- b1 ---- b2 ----b3 ----b5
  #  Fixed    RevX   RevY    RevZ   PrismZ

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
  q = map(list, mbc.q)
  q[1] = [np.pi/2.]
  q[2] = [-np.pi/4.]
  q[3] = [-np.pi/2.]
  q[4] = [0.5]
  mbc.q = q
  rbd.forwardKinematics(mb, mbc)

  X_s = sva.PTransformd(sva.RotY(-np.pi/2.), e.Vector3d(0.1, 0., 0.))
  mbv = MultiBodyViz(mbg, mb, endEffectorDict={'b4':(X_s, 0.1, (0., 1., 0.))})

  # test MultiBodyViz
  from tvtk.tools import ivtk
  viewer = ivtk.viewer()
  mbv.addActors(viewer.scene)
  mbv.display(mb, mbc)

  # test axis
  from axis import Axis
  a1 = Axis()
  a1.addActor(viewer.scene)
  a1.X = sva.PTransformd(sva.RotX(np.pi/2.), e.Vector3d.UnitX())
