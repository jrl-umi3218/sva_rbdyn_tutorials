import numpy as np

import eigen3 as e
import rbdyn as rbd


def multi_task_ik(mb, mbc, tasks, delta=1., maxIter=100, prec=1e-8):
  """
  The multi_task_ik function is a generator that will return at each call
  the new step in the IK process.

  Parameters:
    - mb: MultiBody system.
    - mbc: Initial configuration
    - tasks: List of tasks could be of two form:
      - (weight, task): apply a global weight on all task dimension
      - ((weight,), task): apply a different weight on each dimension of the
        task
    - delta: Integration step
    - maxIter: maximum number of iteration
    - prec: stop the IK if \| \alpha \|_{\inf} < prec

  Returns:
    - Current iteration number
    - Current articular position vector q
    - Current articular velocity vector alpha (descent direction)
    - \| \alpha \|_{\inf}
  """
  q = e.toNumpy(rbd.paramToVector(mb, mbc.q))
  iterate = 0
  minimizer = False

  # transform user weight into a numpy array
  tasks_np = []
  for w, t in tasks:
    dim = t.dimension()
    w_np = np.zeros((dim,1))
    if isinstance(w, (float, int)):
      w_np[:,0] = [w]*dim
    elif hasattr(w, '__iter__'):
      w_np[:,0] = w
    else:
      raise RuntimeError('%s unknow type for weight vector')
    tasks_np.append((w_np, t))

  while iterate < maxIter and not minimizer:
    # compute task data
    gList = map(lambda (w, t): np.mat(w*np.array(t.g(mb, mbc))), tasks_np)
    JList = map(lambda (w, t): np.mat(w*np.array(t.J(mb, mbc))), tasks_np)

    g = np.concatenate(gList)
    J = np.concatenate(JList)

    # compute alpha
    alpha = -np.mat(np.linalg.lstsq(J, g)[0])

    # integrate and run the forward kinematic
    mbc.alpha = rbd.vectorToDof(mb, e.toEigenX(alpha))
    rbd.eulerIntegration(mb, mbc, delta)
    rbd.forwardKinematics(mb, mbc)

    # take the new q vector
    q = e.toNumpy(rbd.paramToVector(mb, mbc.q))

    alphaInf = np.linalg.norm(alpha, np.inf)
    yield iterate, q, alpha, alphaInf # yield the current state

    # check if the current alpha is a minimizer
    if alphaInf < prec:
        minimizer = True
    iterate += 1


if __name__ == '__main__':
  import sys
  sys.path += [".."]

  import spacevecalg as sva

  from ik_tasks import BodyTask, PostureTask, CoMTask
  from robots import TutorialTree

  mbg, mb, mbc = TutorialTree()
  quat = e.Quaterniond(np.pi/3., e.Vector3d(0.1, 0.5, 0.3).normalized())
  mbc.q = [[],
           [3.*np.pi/4.],
           [np.pi/3.],
           [-3.*np.pi/4.],
           [0.],
           [quat.w(), quat.x(), quat.y(), quat.z()]]
  rbd.forwardKinematics(mb, mbc)
  rbd.forwardVelocity(mb, mbc)

  # target frame
  X_O_T = sva.PTransformd(sva.RotY(np.pi/2.), e.Vector3d(1.5, 0.5, 1.))
  X_b5_ef = sva.PTransformd(sva.RotX(-np.pi/2.), e.Vector3d(0., 0.2, 0.))

  # create the task
  bodyTask = BodyTask(mb, mbg.bodyIdByName("b5"), X_O_T, X_b5_ef)
  postureTask = PostureTask(mb, map(list, mbc.q))
  comTask = CoMTask(mb, rbd.computeCoM(mb, mbc) + e.Vector3d(0., 0.5, 0.))

  tasks = [(100., bodyTask), ((0., 10000., 0.), comTask), (1., postureTask)]
  q_res = None
  X_O_p_res = None
  alphaInfList = []
  for iterate, q, alpha, alphaInf in\
      multi_task_ik(mb, mbc, tasks, delta=1., maxIter=200, prec=1e-8):
    q_res = q
    alphaInfList.append(alphaInf)

  print 'iter number', len(alphaInfList)
  print 'last alpha norm', alphaInfList[-1]
  print
  print 'bodyTask error:', bodyTask.g(mb, mbc).T
  print 'postureTask error:', postureTask.g(mb, mbc).T
  print 'comTask error:', comTask.g(mb, mbc).T

