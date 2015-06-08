from tvtk.api import tvtk

import eigen3 as e
import spacevecalg as sva

from transform import setActorTransform

class Axis(object):
  def __init__(self, X=sva.PTransformd.Identity(), length=0.1, text=''):
    """
    Create a 3D axis.
    """
    self._X = X
    self.axesActor = tvtk.AxesActor(total_length=(length,)*3,
                                    axis_labels=False)
    self.axesActor.user_transform = tvtk.Transform()

    textSource = tvtk.TextSource(text=text, backing=False)
    textPdm = tvtk.PolyDataMapper(input=textSource.output)
    #self.textActor = tvtk.Actor(mapper=textPdm)
    self.textActor = tvtk.Follower(mapper=textPdm)
    # take the maximum component of the bound and use it to scale it
    m = max(self.textActor.bounds)
    scale = length/m
    self.textActor.scale = (scale,)*3
    # TODO compute the origin well...
    self.textActor.origin = (
      -(self.textActor.bounds[0] + self.textActor.bounds[1])/2.,
      -(self.textActor.bounds[2] + self.textActor.bounds[3])/2.,
      -(self.textActor.bounds[4] + self.textActor.bounds[5])/2.,
    )
    ySize = self.textActor.bounds[3]*1.2
    self.X_text = sva.PTransformd(e.Vector3d(0., -ySize, 0.))
    self._transform()


  def _transform(self):
    setActorTransform(self.axesActor, self._X)
    # user_transform is not take into account by Follower
    self.textActor.position = tuple((self.X_text*self._X).translation())


  @property
  def X(self):
    return self._X


  @X.setter
  def X(self, X):
    self._X = X
    self._transform()


  def addActors(self, scene):
    """
    Add actors to the scene.
    """
    scene.add_actor(self.axesActor)
    scene.add_actor(self.textActor)
    self.textActor.camera = scene.camera


  def removeActors(self, scene):
    """
    Remove actors from the scene.
    """
    scene.remove_actor(self.axesActor)
    scene.remove_actor(self.textActor)
