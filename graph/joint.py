from tvtk.api import tvtk

import eigen3 as e
import spacevecalg as sva


def makeActor(source, color):
  """
  Create an actor from a source and a color.
  """
  pdm = tvtk.PolyDataMapper(input=source.output)
  actor = tvtk.Actor(mapper=pdm)
  actor.property.color = color
  actor.user_transform = tvtk.Transform()
  return actor


def revoluteJoint(joint):
  """
  Return a cylinder and the appropriate static transform.
  """
  axis = e.toEigen(e.toNumpy(joint.motionSubspace())[:3])
  s = tvtk.CylinderSource(height=0.1, radius=0.02)
  quat = e.Quaterniond()
  # Cylinder is around the Y axis
  quat.setFromTwoVectors(axis, e.Vector3d.UnitY())
  return makeActor(s, tuple(axis)), sva.PTransformd(quat)


def prismaticJoint(joint):
  """
  Return a prism and the appropriate static transform.
  """
  axis = e.toEigen(e.toNumpy(joint.motionSubspace())[3:])
  s = tvtk.CubeSource(x_length=0.02, y_length=0.1, z_length=0.02)
  quat = e.Quaterniond()
  quat.setFromTwoVectors(axis, e.Vector3d.UnitY())
  return makeActor(s, tuple(axis)), sva.PTransformd(quat)


def sphericalJoint(joint):
  """
  Return a shpere and the appropriate static transform.
  """
  s = tvtk.SphereSource(radius=0.02)
  return makeActor(s, (1., 1., 1.)), sva.PTransformd.Identity()
