import pickle
from xml.dom.minidom import parseString

import numpy as np

import eigen3 as e
import spacevecalg as sva
import rbdyn as rbd

import rbdyn_urdf

# allow to pickle sva and rbd structure
rbd.copy_reg_pickle()


def urdfToPickle(urdfStr, outPath, packageToPath, extensionDict):
  """
  Save an urdf into a pickle file.

  Parameters:
    - urdfStr: urdf string
    - outPath: output path to save the pickle file
    - packageToPath: dict {pkg:path,..} to replace urdf mesh path
      that depend of the ros package:// url
    - extensionDict: dict {oldExt:newExt, ...} to replace urdf mesh path
      extension

  Pickle file tuple:
    - mb: MultiBody
    - meshDict: {bodyName:(meshPath, X_s, scale}
    -- X_s: static transform from link origin
  """
  urdfData = rbdyn_urdf.readUrdf(urdfStr)
  # copy urdfData.mbg (in urdfData structure) to be able to modify it
  mbg = urdfData.mbg
  baseLinkId = mbg.bodyIdByName('base_link')
  mb = mbg.makeMultiBody(baseLinkId, True)

  meshDict = linkVisual(mb, urdfStr, packageToPath, extensionDict)

  with open(outPath, 'w') as outFile:
    pickle.dump((mb, meshDict), outFile)


def attrToList(attr):
  return map(float, attr.split())


def attrToVector(attr):
  return e.Vector3d(*attrToList(attr))


def getAttributeDefault(dom, attr, default):
  if dom.hasAttribute(attr):
    return dom.getAttribute(attr)
  else:
    return default


def RPY(r, p, y):
  ca1 = np.cos(y)
  sa1 = np.sin(y)
  cb1 = np.cos(p)
  sb1 = np.sin(p)
  cc1 = np.cos(r)
  sc1 = np.sin(r)
  m = np.matrix([[ca1*cb1,ca1*sb1*sc1 - sa1*cc1,ca1*sb1*cc1 + sa1*sc1],
                 [sa1*cb1,sa1*sb1*sc1 + ca1*cc1,sa1*sb1*cc1 - ca1*sc1],
                 [-sb1,cb1*sc1,cb1*cc1]])
  return e.toEigen(m.T)


def XFromOrigin(dom):
  X = sva.PTransform.Identity()

  originDom = dom.getElementsByTagName('origin')
  if len(originDom) == 1:
    r = attrToVector(getAttributeDefault(originDom[0], 'xyz', '0 0 0'))
    E = RPY(*attrToList(getAttributeDefault(originDom[0], 'rpy', '0 0 0')))
    X = sva.PTransform(E, r)

  return X


def meshPathAndScaleFromGeometry(dom):
  geometryDom = dom.getElementsByTagName('geometry')
  meshDom = geometryDom[0].getElementsByTagName('mesh')
  fileName = meshDom[0].getAttribute('filename')
  scale = getAttributeDefault(meshDom[0], 'scale', "1 1 1")
  return fileName, attrToList(scale)


def replacePackage(fileName, packageToPath):
  for pkg, path in packageToPath.items():
    fileName = fileName.replace('package://%s' % pkg, path)
  return fileName


def replaceExt(fileName, extensionDict):
  for oldExt, newExt in extensionDict.items():
    fileName = fileName.replace('.%s' % oldExt, '.%s' % newExt)
  return fileName


def linkVisual(mb, urdfStr, packageToPath, extensionDict):
  dom = parseString(urdfStr)

  bodyNames = {b.name() for b in mb.bodies()}
  linkDom = filter(lambda bDom: bDom.getAttribute('name') in bodyNames,
                   dom.getElementsByTagName('link'))

  meshDict = {}
  for bDom in linkDom:
    for vDom in bDom.getElementsByTagName('visual'):
      X_s = XFromOrigin(vDom)
      try:
        meshFileName, scale = meshPathAndScaleFromGeometry(vDom)
        meshFileNameNoPkg = replacePackage(meshFileName, packageToPath)
        meshFileNameNoPkgNewExt = replaceExt(meshFileNameNoPkg, extensionDict)
        meshDict[bDom.getAttribute('name')] =\
          (meshFileNameNoPkgNewExt, X_s, scale)
      except IndexError:
        pass

  return meshDict


def mbgFromMb(mb):
  """
  Construct a MultiBodyGraph from a MultiBody.
  """
  mbg = rbd.MultiBodyGraph()

  for b in mb.bodies():
    mbg.addBody(b)

  joints = list(mb.joints())[1:]
  for j in joints:
    mbg.addJoint(j)

  I = sva.PTransformd.Identity()
  for jIndex, j in enumerate(joints, 1):
    predIndex = mb.predecessor(jIndex)
    predId = mb.body(predIndex).id()
    succIndex = mb.successor(jIndex)
    succId = mb.body(succIndex).id()

    mbg.linkBodies(predId, mb.transform(jIndex),
                   succId, I, j.id())

  return mbg



if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()

  def dictType(string):
    commaList = string.split(',')
    return dict(map(lambda cl: cl.split(':'), commaList))

  parser.add_argument('urdfFile', help='path to the input urdf file', type=file)
  parser.add_argument('picklePath', help='path to the output pickle file',
                      type=str)
  parser.add_argument('--pkgToPath',
                      help='path to pkg name (pkgName:path,pkgName:path,...)',
                      type=dictType, default={})
  parser.add_argument('--extensionDict',
                      help='change mesh extension (extIn:extOut,extIn:extOut,...)',
                      type=dictType, default={})
  args = parser.parse_args()

  urdfStr = args.urdfFile.read()
  urdfToPickle(urdfStr, args.picklePath, args.pkgToPath, args.extensionDict)
