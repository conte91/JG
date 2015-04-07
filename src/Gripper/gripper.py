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
    return "Shape(" + ' '.join([repr(element) for element in self.elements])+")"

  def collisionScore(self,otherShape):
    name0 = type(self)
    name1 = type(otherShape)
    names = sorted((shape1,shape2))

    if names == ("Cube","Cube"):
      thisCube = self.vertices
      otherCube = otherShape.vertices

      x0 = min(thisCube[0],otherCube[0])
      y0 = min(thisCube[1],otherCube[1])
      z0 = min(thisCube[2],otherCube[2])

      x1 = max(thisCube[0],otherCube[0])
      y1 = max(thisCube[1],otherCube[1])
      z1 = max(thisCube[2],otherCube[2])

      a0 = min(thisCube[0],otherCube[0])
      b0 = min(thisCube[1],otherCube[1])
      c0 = min(thisCube[2],otherCube[2])

      a1 = max(thisCube[0],otherCube[0])
      b1 = max(thisCube[1],otherCube[1])
      c1 = max(thisCube[2],otherCube[2])

      x = max(min(a1,x1)-max(a0,x0),0)
      y = max(min(b1,y1)-max(b0,y0),0)
      z = max(min(c1,z1)-max(c0,z0),0)

      print "test me"
      overlap = sqrt(x*y*z)
      return overlap

    elif names == ("cube","sphere"):

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

    elif names == ("sphere","sphere"):

      c0 = self.center
      r0 = self.radius
      c1 = otherShape.center
      r1 = otherShape.radius

      coveredSpace = r0 + r1
      spaceBetween = distance(c0,c1) 
      intersectionLen = spaceBetween - coveredSpace
      return max(0,intersectionLen)


class Cube(Shape):
  def __init__(self,vertices):
    (v1,v2) = vertices
    v1 = (min(v1[0],v2[0]),max(v1[0],v2[0]))
    v2 = (min(v1[1],v2[1]),max(v1[1],v2[1]))
    self.vertices = vertices
    self.shapeName = "cube"

  def __repr__(self):
    return "Cube(" + vertices[0] + "," + vertices[1] + ")"


class Sphere(object):
  def __init__(self,center=point,radius=radius):
    self.center = center
    self.radius = radius
    self.shapeName = "sphere"

  def __repr__(self):
    return "Sphere(" + self.center + "," + self.radius + ")"

  def distance(p0,p1):
    x0,y0 = p0
    x1,y1 = p1
    return sqrt((x0 - x1)**2 + (y0 - y1)**2)


class RobotData(object):
  # fake
  def __init__(self,items):
    self.items = items

  def __repr__(self):
    return '\n'.join([repr(item) for item in self.items[:5]])

  def getBin(self):
    return KivaBin(self.items)

  def removeItem(self,item):
    ind = items.index(item)
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
    ind = items.index(item)
    self.items.pop(ind)

  def getItemPose(self,itemName):
    dx,dy,dz,da,db,dg = [100*random() for i in range(6)]
    return {'x':dx,'y':dy,'z':dz,'a':da,'b':db,'g':dg}


class Item(object):
  def __init__(self, name, pose, roughShape, shape, grasps):
    self.name = name
    self.pose = pose
    self.roughShape = roughShape
    self.shape = shape
    self.grasps = grasps

  def __hash__(self):
    return hash(self.name)

  def __repr__(self):
    return "Item("+self.name+")"

  def __eq__(self, other):
    return (self.pose, self.shape) == (other.pose, other.shape)

  def transform(self, transformationMatrix):
    for key in transformationMatrix.keys():
      self.pose[key] += transformationMatrix[key]

  def getGrasps(self):
    return self.grasps


class Grasp(object):
  def __init__(self, approachPose, gripPose, minForce, maxForce):
    self.approachPose = approachPose
    self.gripPose = gripPose
    self.roughShape = Shape([((1,2,0),(3,5,5))])
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
    score.append(calculateGraspScore(grasp,targetItem,targetBin))
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
  originXYZ = [0, 0, 0 ]

  objects = [
    ['munchkin_white_hot_duck_bath_toy',(150,160,80)],
    ['mark_twain_huckleberry_finn',(128,250,16)],
    ['sharpie_accent_tank_style_highlighters',(120,130,20)],
    ['genuine_joe_plastic_stir_sticks',(144,97,108)],
    ['safety_works_safety_glasses',(220,55,115)],
    ['rollodex_mesh_collection_jumbo_pencil_cup', (100,136,100)],
    ['dr_browns_bottle_brush',(350,140,50)],
    ['kygen_squeakin_eggs_plush_puppies',(240,60,130)],
    ['kong_duck_dog_toy',(170,110,70)],
    ['mommys_helper_outlet_plugs',(190,60,90)],
    ['highland_6539_self_stick_notes',(150,40,50)],
    ['expo_dry_erase_board_eraser',(130,35,55)],
    ['paper_mate_12_count_mirado_black_warrior',(195,20,50)],
    ['laugh_out_loud_joke_book',(180,10,110)],
    ['stanley_66_052',(195,20,100)],
    ['mead_index_cards',(130,78,23)],
    ['elmers_washable_no_run_school_glue', (65,150,35)],
  ]

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

  dummyGrasp = Grasp(originPose, originPose, 0, 100000)
  cubes = [((1, 2, 0), (2, 0, 1)),]
  dummyShape = Shape(cubes)
  dummyPose = originPose

  items = []
  for i in range(12):
    grasps = []
    for j in range(4):
      grasps.append(dummyGrasp)
    if i < len(objects):
      name = objects[i][0]
      cube0 = (originXYZ, objects[i][1])
      cube1 = cube0
      cube2 = cube0
      cube3 = cube0
      cubes = [cube1,]
      roughShape = Shape(cubes)
      cubes = [cube0, cube1, cube2, cube3,]
      fineShape = Shape(cubes)

      someItem = Item(name, originPose, roughShape, fineShape, grasps)
    else:
      someItem = Item(str(i*j), originPose, dummyShape, dummyShape, grasps)

    items.append(someItem)

  targetItem = items[0]

  # fake
  robotData = RobotData(items)



  kivaBin = KivaBin(items)

  leftoverBin = deepcopy(kivaBin)
  leftoverBin.removeItem(targetItem)

  score = calculateGraspScore(grasps[0], items[0], leftoverBin)
  getBestGrasp(targetItem, kivaBin)
  print "score",score
