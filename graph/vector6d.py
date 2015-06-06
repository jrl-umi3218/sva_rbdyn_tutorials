from tvtk.api import tvtk

import eigen3 as e
import spacevecalg as sva

from transform import setActorTransform

class Vector6dViz(object):
  def __init__(self, linear, angular, frame, linColor, angColor):
    """
    Create the visualization of a 6D vector with a linear and angular part.
    Parameter:
      linear: 3d linear component (e.Vector3d)
      angular: 3d angular component (e.Vector3d)
      frame: vector frame (sva.PTransformd)
      linColor: linear component color (float, float, float)
      angColor: angular component color (float, float, float)
    """
    self.linearActor, X_l = self._createVector(linear, frame, linColor)
    self.angularActor, X_a = self._createVector(angular, frame, angColor)

    # create a Arc around the angular axis
    # The arc must turn around the X axis (Arrow default axis)
    angNorm = angular.norm()
    angNormW = angNorm*0.3
    arcSource = tvtk.ArcSource(point1=(angNorm/2., -angNormW, -angNormW),
                               point2=(angNorm/2., -angNormW, angNormW),
                               center=(angNorm/2., 0., 0.), negative=True,
                               resolution=20)
    arcPdm = tvtk.PolyDataMapper(input=arcSource.output)
    self.arcActor = tvtk.Actor(mapper=arcPdm)
    self.arcActor.property.color = angColor
    self.arcActor.user_transform = tvtk.Transform()
    # apply the angular transform
    setActorTransform(self.arcActor, X_a)


  def _createVector(self, vector, frame, color):
    source = tvtk.ArrowSource()
    pdm = tvtk.PolyDataMapper(input=source.output)
    actor = tvtk.Actor(mapper=pdm)
    actor.user_transform = tvtk.Transform()
    actor.property.color = color
    norm = vector.norm()
    actor.scale = (norm,)*3
    quat = e.Quaterniond()
    # arrow are define on X axis
    quat.setFromTwoVectors(vector, e.Vector3d.UnitX())
    X = sva.PTransformd(quat)*frame
    setActorTransform(actor, X)
    return actor, X


  def addActors(self, scene):
    """
    Add actors to the scene.
    """
    scene.add_actor(self.linearActor)
    scene.add_actor(self.angularActor)
    scene.add_actor(self.arcActor)


  def removeActors(self, scene):
    """
    Remove actors from the scene.
    """
    scene.remove_actor(self.linearActor)
    scene.remove_actor(self.angularActor)
    scene.remove_actor(self.arcActor)



class ForceVecViz(Vector6dViz):
  def __init__(self, F, frame):
    """
    Helper class to display a sva.ForceVecd.
    """
    super(ForceVecViz, self).__init__(F.force(), F.couple(), frame,
                                      (1., 0., 0.), (1., 1., 0.))



class MotionVecViz(Vector6dViz):
  def __init__(self, M, frame):
    """
    Helper class to display a sva.MotionVecd.
    """
    super(MotionVecViz, self).__init__(M.linear(), M.angular(), frame,
                                       (0., 0., 1.), (1., 0., 1.))
