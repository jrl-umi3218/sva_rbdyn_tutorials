import os
import pickle

import rbdyn as rbd

from urdf_to_pickle import mbgFromMb

def ReemC():
  """
  Return the ReemC MultiBodyGraph, MultiBody, the zeroed MultiBodyConfig
  and the mesh dictionary.
  """
  rbd.copy_reg_pickle()

  reemcDataPath = os.path.join(os.path.dirname(__file__), 'reemc_data')
  filePath = os.path.join(reemcDataPath, 'reemc_full.pkl')
  with open(filePath, 'r') as reemCData:
    mb, meshDict = pickle.load(reemCData)
    mbc = rbd.MultiBodyConfig(mb)
    mbc.zero(mb)

    mbg = mbgFromMb(mb)

    def fixMeshPath(path):
      return os.path.join(reemcDataPath, path)

    meshDictFixPath = dict(map(lambda (key,(path,X,scale)):
                               (key,(fixMeshPath(path),X,scale)),
                               meshDict.items()))

    return mbg, mb, mbc, meshDictFixPath
