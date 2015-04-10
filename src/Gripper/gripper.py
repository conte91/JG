#import RobotData as robotData
from copy import deepcopy
import numpy as np
from math import sqrt
from random import random


GRASP_ERROR_LIMIT = 1

class Shape(object):
  def __init__(self,elements):
    self.elements = elements

  def __repr__(self):
    return 'Shape(\n  ' + '\n  '.join([repr(element) for element in self.elements]) + '\n)'

  def intersect(self,otherShape):
    total = 0
    for e0 in self.elements:
      for e1 in otherShape.elements:
        total += e0.getCollisionScore(e1)
    return total



class PrimitiveShape(object):
  def __init__(self):
    print "primitive shape can only be extended"
    raise Exception("can't init PrimitiveShape")

  def getCollisionScore(self,otherShape):
    name0 = type(self).__name__
    name1 = type(otherShape).__name__
    names = sorted((name0,name1))

    if names == ["Cuboid","Cuboid"]:
      thisCuboid = self.vertices
      otherCuboid = otherShape.vertices
      x,y,z = range(3)

      x0 = min(thisCuboid[0][x],otherCuboid[0][x])
      y0 = min(thisCuboid[0][y],otherCuboid[0][y])
      z0 = min(thisCuboid[0][z],otherCuboid[0][z])

      x1 = max(thisCuboid[0][x],otherCuboid[0][x])
      y1 = max(thisCuboid[0][y],otherCuboid[0][y])
      z1 = max(thisCuboid[0][z],otherCuboid[0][z])

      a0 = min(thisCuboid[1][x],otherCuboid[1][x])
      b0 = min(thisCuboid[1][y],otherCuboid[1][y])
      c0 = min(thisCuboid[1][z],otherCuboid[1][z])

      a1 = max(thisCuboid[1][x],otherCuboid[1][x])
      b1 = max(thisCuboid[1][y],otherCuboid[1][y])
      c1 = max(thisCuboid[1][z],otherCuboid[1][z])

      x = max(max(a0,x0)-min(a1,x1),0)
      y = max(max(b0,y0)-min(b1,y1),0)
      z = max(max(c0,z0)-min(c1,z1),0)

      overlap = pow(x*y*z,1/3)
      return overlap

    elif names == ["Cuboid","Sphere"]:

      (v1,v2) = self.vertices
      (c, r) = (otherShape.center, otherShape.radius)

      # assume C1 and C2 are element-wise sorted, if not, do that now

      if c[0] < v1[0]:
        #the sphere center is left of cube
        r -= abs(c[0] - v1[0])
      elif c[0] > v2[0]:
        #the sphere center is right of the cube
        r -= abs(c[0] - v2[0])
      #if it's inside, do nothing

      if c[1] < v1[1]:
        r -= abs(c[1] - v1[1])
      elif c[1] > v2[1]:
        r -= abs(c[1] - v2[1])

      if c[2] < v1[3]:
        r -= abs(c[2] - v1[3])
      elif c[2] > v2[3]:
        r -= abs(c[2] - v2[3])

      #r = r - |dx| + |dy| + |dz|
      return max(r,0)

    elif names == ["Sphere","Sphere"]:

      c0 = self.center
      r0 = self.radius
      c1 = otherShape.center
      r1 = otherShape.radius

      coveredSpace = r0 + r1
      spaceBetween = distance(c0,c1) 
      intersectionLen = spaceBetween - coveredSpace
      return max(0,intersectionLen)


class Cuboid(PrimitiveShape):
  def __init__(self,v0,v1):
    tv0 = v0
    tv1 = v1
    v1 = (
        min(tv0[0],tv1[0]),
        min(tv0[1],tv1[1]),
        min(tv0[2],tv1[2])
        )
    v1 = (
        max(tv0[0],tv1[0]),
        max(tv0[1],tv1[1]),
        max(tv0[2],tv1[2])
        )
    self.vertices = (v0,v1)

  def __repr__(self):
    return 'Cuboid(' + ' '.join([repr(vertex) for vertex in self.vertices]) + ')'

  def __name__(self):
    return 'Cuboid'


class Sphere(PrimitiveShape):
  def __init__(self,center,radius):
    self.center = center
    self.radius = radius
    self.shapeName = "sphere"

  def __repr__(self):
    return "Sphere(" + self.center + "," + self.radius + ")"

  def distance(p0,p1):
    x0,y0,z0 = p0
    x1,y1,z1 = p1
    squares = (x0 - x1)**2 + (y0 - y1)**2 + (z0 - z1)**2
    return pow(squares,1/3)


class RobotData(object):
  # fake
  def __init__(self,items):
    self.items = items

  def __repr__(self):
    return '\n'.join([repr(item) for item in self.items[:5]])

  def getBin(self):
    return KivaBin(self.items)

  def removeItem(self,item):
    ind = self.items.index(item)
    self.items.pop(ind)


class KivaBin(object):
  # fake
  def __init__(self,items):
    self.items = items

  def __repr__(self):
    return "KivaBin(\n\t" + '\n\t'.join([repr(item) for item in self.items[:5]]) + "\n)"

  def getBinItems(self):
    return self.items

  def removeItem(self,item):
    ind = self.items.index(item)
    self.items.pop(ind)

  def getItemPose(self,itemName):
    dx,dy,dz,da,db,dg = [100*random() for i in range(6)]
    return {'x':dx,'y':dy,'z':dz,'a':da,'b':db,'g':dg}


class Item(object):
  def __init__(self, name, pose, roughShape, fineShape, grasps):
    self.name = name
    self.pose = pose
    self.roughShape = roughShape
    self.fineShape = fineShape
    self.grasps = grasps

  def __hash__(self):
    return hash(hash(self.name) + sum(self.pose.values()))

  def __repr__(self):
    return "Item("+self.name+" at " + repr(self.pose) + ")"

  def __eq__(self, other):
    return (self.name, self.pose) == (other.name, other.pose)

  def transform(self, transformationMatrix):
    for key in transformationMatrix.keys():
      self.pose[key] += transformationMatrix[key]

  def getGrasps(self):
    return self.grasps


class Grasp(object):
  def __init__(self, approachPose, gripPose, minForce, maxForce):
    self.approachPose = approachPose
    self.gripPose = gripPose

    print "using fake shape"
    self.roughShape = Shape([
      Cuboid((1,2,0),(3,5,5)),
      ])
    self.minForce = minForce
    self.maxForce = maxForce

  def __repr__(self):
    return "Grasp(" + ' '.join([str(val) for val in self.gripPose.values()]) + ")"

  def getCollisionVolume(self, otherItem):
    volume = self.roughShape.intersect(otherItem.roughShape)
    return volume

def getBestGrasp(targetItem,targetBin):

  transformationMatrix = targetBin.getItemPose(targetItem)
  targetItem.transform(transformationMatrix)

  score = []
  grasps = targetItem.getGrasps()
  for grasp in grasps:
    score.append(calculateGraspScore(grasp, targetItem, targetBin))
  best = score.index(max(score))
  return best

def calculateGraspScore(grasp, targetItem, targetBin):
  leftoverBin = deepcopy(targetBin)
  leftoverBin.removeItem(targetItem)

  if not isDoable(grasp, targetItem):
    return -1

  scores = dict()
  collisionVolume = 1
  for binItem in leftoverBin.getBinItems():
    scores[binItem] = grasp.getCollisionVolume(binItem)
    collisionVolume += scores[binItem]

  bestScore = 1.0
  if collisionVolume == 0:
    return bestScore
  else:
    return bestScore/collisionVolume


def isDoable(grasp, targetItem):
  """
  Checks if the gripper can grasp the item ignoring the environment
  we only allow GRASP_ERROR_LIMIT error
  """
  runningSum = 0
  for key in grasp.gripPose.keys():
    runningSum += ( grasp.gripPose[key] + targetItem.pose[key] ) ** 2
  distance = np.sqrt(runningSum)
  if distance > GRASP_ERROR_LIMIT:
    return False
  return True

if __name__ == '__main__':
  grasps = {}
  gripper = "somegripper"
  originPose = {'x': 0, 'y': 0, 'z': 0, 'a': 0, 'b': 0, 'g': 0, }
  originXYZ = (0, 0, 0)

  objects = {
    'munchkin_white_hot_duck_bath_toy': (150,160,80),
    'mark_twain_huckleberry_finn': (128,250,16),
    'sharpie_accent_tank_style_highlighters': (120,130,20),
    'genuine_joe_plastic_stir_sticks': (144,97,108),
    'safety_works_safety_glasses': (220,55,115),
    'rollodex_mesh_collection_jumbo_pencil_cup':  (100,136,100),
    'dr_browns_bottle_brush': (350,140,50),
    'kygen_squeakin_eggs_plush_puppies': (240,60,130),
    'kong_duck_dog_toy': (170,110,70),
    'mommys_helper_outlet_plugs': (190,60,90),
    'highland_6539_self_stick_notes': (150,40,50),
    'expo_dry_erase_board_eraser': (130,35,55),
    'paper_mate_12_count_mirado_black_warrior': (195,20,50),
    'laugh_out_loud_joke_book': (180,10,110),
    'stanley_66_052': (195,20,100),
    'mead_index_cards': (130,78,23),
    'elmers_washable_no_run_school_glue': (65,150,35),
  }

  """
    ['champion_copper_plus_spark_plug',],
    ['cheezit_big_original',],
    ['crayola_64_ct',],
    ['dove_beauty_bar',],
    ['feline_greenies_dental_treats',],
    ['first_years_take_and_toss_straw_cups',],
    ['kong_air_dog_squeakair_tennis_ball',],
    ['kong_sitting_frog_dog_toy',],
    ['one_with_nature_soap_dead_sea_mud',],
    ['oreo_mega_stuf',],
  ]
  """

  items = dict()
  print "fineShape is fake"
  dummyGrasp = Grasp(originPose, originPose, 0, 100000)
  print "using dummy grasps"
  fakeCuboids = [
      Cuboid(originXYZ, (1, 2, 1)),
      Cuboid(originXYZ, (2, 2, 3)),
    ]

  for name in objects.keys():
    roughShape = Shape([
      Cuboid( originXYZ , objects[name] )
      ])
    fineShape = Shape(fakeCuboids)
    grasps = []
    for j in range(4):
      grasps.append(dummyGrasp)
    items[name] = Item(name, originPose, roughShape, fineShape, grasps);

  dummyShape = Shape(fakeCuboids)
  dummyPose = originPose

  targetItem = items.values()[0]

  # fake
  robotData = RobotData(items.values())
  kivaBin = KivaBin(items.values())

  score = calculateGraspScore(targetItem.grasps[0], targetItem, kivaBin)
  getBestGrasp(targetItem, kivaBin)
  print "score",score
