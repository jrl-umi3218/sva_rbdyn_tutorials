from tvtk.api import tvtk

import spacevecalg as sva

from transform import setActorTransform

class Axis(object):
  def __init__(self, X=sva.PTransformd.Identity(), length=0.1):
    """
    Create a 3D axis.
    """
    self._X = X
    self.actor = tvtk.AxesActor(total_length=(length,)*3)
    self.actor.user_transform = tvtk.Transform()


  def _transform(self):
    setActorTransform(self.actor, self._X)


  @property
  def X(self):
    return self._X


  @X.setter
  def X(self, X):
    self._X = X
    self._transform()


  def addActor(self, scene):
    """
    Add the AxesActor to the scene.
    """
    scene.add_actor(self.actor)


  def removeActor(self, scene):
    scene.remove_actor(self.actor)
