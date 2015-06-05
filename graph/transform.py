def vtkTransform(X):
  """
  Transform a PTranform into a vtk homogeneous transformation matrix.
  """
  R = X.rotation()
  T = X.translation()
  return (R[0], R[1], R[2], T[0],
          R[3], R[4], R[5], T[1],
          R[6], R[7], R[8], T[2],
          0.,   0.,   0.,   1.)


def setActorTransform(actor, X):
  """
  Set an actor user_transform with a PTransform
  """
  actor.user_transform.set_matrix(vtkTransform(X))
