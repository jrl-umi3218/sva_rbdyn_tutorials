from os.path import splitext

from tvtk.api import tvtk

import eigen3 as e
import spacevecalg as sva



FILE_READER = {
  '.obj': tvtk.OBJReader,
  '.stl': tvtk.STLReader,
}


def linesBody(mb, bodyId, successorJointsId):
  """
  Return a mesh represented by lines and the appropriate static transform.
  """
  apd = tvtk.AppendPolyData()
  sources = []

  # create a line from the body base to the next joint
  for s in map(mb.jointIndexById, successorJointsId[bodyId]):
    X_s = mb.transform(s)
    sources.append(tvtk.LineSource(point1=(0., 0., 0.),
                                   point2=tuple(X_s.translation())))

  # add an empty source to avoid a warning if AppendPolyData have 0 source
  if len(sources) == 0:
    sources.append(tvtk.PointSource(radius=0.))

  map(lambda s: apd.add_input(s.output), sources)
  apd.update()

  pdm = tvtk.PolyDataMapper(input=apd.output)
  actor = tvtk.Actor(mapper=pdm)
  actor.property.color = (0., 0., 0.)
  actor.user_transform = tvtk.Transform()

  return actor, sva.PTransformd.Identity()


def meshBody(fileName, scale=(1., 1., 1.)):
  """
  Return a mesh actor and the appropriate static transform.
  """
  reader = FILE_READER[splitext(fileName)[1]](file_name=fileName)
  output = reader.output

  # if a scale is set we have to apply it
  if map(float, scale) != [1., 1., 1.]:
    tpdf_transform = tvtk.Transform()
    tpdf_transform.identity()
    tpdf_transform.scale(scale)
    tpdf = tvtk.TransformPolyDataFilter(input=reader.output, transform=tpdf_transform)
    tpdf.update()
    output = tpdf.output

  # compute mesh normal to have a better render and reverse mesh normal
  # if the scale flip them
  pdn = tvtk.PolyDataNormals(input=output)
  pdn.update()
  output = pdn.output

  pdm = tvtk.PolyDataMapper(input=output)
  actor = tvtk.Actor(mapper=pdm)
  actor.user_transform = tvtk.Transform()

  return actor, sva.PTransformd.Identity()


def endEffectorBody(X_s, size, color):
  """
  Return a end effector reprsented by a plane
  and the appropriate static transform.
  """
  apd = tvtk.AppendPolyData()

  ls = tvtk.LineSource(point1=(0., 0., 0.),
                       point2=tuple(X_s.translation()))

  p1 = (sva.PTransformd(e.Vector3d.UnitX()*size)*X_s).translation()
  p2 = (sva.PTransformd(e.Vector3d.UnitY()*size)*X_s).translation()
  ps = tvtk.PlaneSource(origin=tuple(X_s.translation()),
                        point1=tuple(p1),
                        point2=tuple(p2),
                        center=tuple(X_s.translation()))

  apd.add_input(ls.output)
  apd.add_input(ps.output)

  pdm = tvtk.PolyDataMapper(input=apd.output)
  actor = tvtk.Actor(mapper=pdm)
  actor.property.color = color
  actor.user_transform = tvtk.Transform()

  return actor, sva.PTransformd.Identity()
