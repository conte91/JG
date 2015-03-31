#import RobotData as robotData
from copy import deepcopy
import numpy as np


GRASP_ERROR_LIMIT = 1

class Shape(object):
  def __init__(self,cubes):
    self.cubes = cubes

  def intersect(self,otherShape):
    for myCube in self.cubes:
      for otherCube in otherShape.cubes:
        x0 = min(myCube[0][0],myCube[1][0])
        y0 = min(myCube[0][1],myCube[1][1])
        z0 = min(myCube[0][2],myCube[1][2])

        x1 = max(myCube[0][0],myCube[1][0])
        y1 = max(myCube[0][1],myCube[1][1])
        z1 = max(myCube[0][2],myCube[1][2])

        a0 = min(myCube[0][0],myCube[1][0])
        b0 = min(myCube[0][1],myCube[1][1])
        c0 = min(myCube[0][2],myCube[1][2])

        a1 = max(myCube[0][0],myCube[1][0])
        b1 = max(myCube[0][1],myCube[1][1])
        c1 = max(myCube[0][2],myCube[1][2])

        x = max(min(a1,x1)-max(a0,x0),0)
        y = max(min(b1,y1)-max(b0,y0),0)
        z = max(min(c1,z1)-max(c0,z0),0)

        print "test me"
        overlap = x*y*z
    return overlap


class RobotData(object):
  # fake
  def __init__(self,items):
    self.items = items

  def getBinItems(self):
    return self.items

  def removeItem(self,item):
    ind = items.index(item)
    self.items.pop(ind)


class Item(object):
  def __init__(self, name, pose, roughShape, shape, grasps):
    self.name = name
    self.pose = pose
    self.roughShape = roughShape
    self.shape = shape
    self.grasps = grasps

  def __hash__(self):
    return hash(self.name)

  def __eq__(self, other):
    return (self.pose, self.shape) == (other.pose, other.shape)

  def translate(self, translationMatrix):
    self.pose += translationMatrix
    self.roughShape += translationMatrix

  def getGrasps(self):
    return self.grasps


class Grasp(object):
  def __init__(self, approachPose, gripPose, minForce, maxForce):
    self.approachPose = approachPose
    self.gripPose = gripPose
    self.minForce = minForce
    self.maxForce = maxForce

  def translate(self, translationMatrix):
    self.approachPose += translationMatrix
    self.gripPose += translationMatrix

  def getCollisionVolume(self, otherItem):
    # missing shape <-> shape collision checking
    points = [((1,2,0),(3,5,5)),]
    roughShape = Shape(points)
    volume = roughShape.intersect(otherItem.roughShape)
    return volume

def getBestGrasp(targetItem):
  binItems = robotData.getBinItems()
  binItems.pop(targetItem)

  translationMatrix = robotData.getItemPose(targetItem)
  targetItem.translate(translationMatrix)

  score = []
  for grasp in targetItem.getGrasps():
    score[grasp] = calculateGraspScore(grasp,)


def calculateGraspScore(grasp, targetItem, targetBin):
  if not isDoable(grasp, targetItem):
    return -1

  scores = dict()
  for binItem in targetBin.getBinItems():
    collisionVolume = 0
    for grasp in targetItem.getGrasps():
      collisionVolume += grasp.getCollisionVolume(binItem)

    scores[binItem] = collisionVolume
  return scores


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
  originPose = {'x': 0, 'y': 0, 'z': 0, 'alpha': 0, 'beta': 0, 'gamma': 0, }

  objects = [
    ['munchkin_white_hot_duck_bath_toy',(150,160,80)],
    ['mark_twain_huckleberry_finn',(128,250,16)],
    ['sharpie_accent_tank_style_highlighters',(120,130,20],
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
    ['laugh_out_loud_joke_book',(180,10,110)]
    ['stanley_66_052',(195,20,100)],
    ['mead_index_cards',(130,78,23)],
    ['elmers_washable_no_run_school_glue', (65,150,35)],

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

  dummyGrasp = Grasp(originPose, originPose, 0, 100000)
  points = [(1, 2, 0), (2, 0, 1)]
  dummyShape = Shape(points)
  dummyPose = originPose

  items = []
  for i in range(12):
    grasps = []
    for j in range(4):
      grasps.append(dummyGrasp)
    someItem = Item(str(i*j),originPose, dummyShape, dummyShape, grasps)
    items.append(someItem)

  targetItem = items[0]

  # fake
  robotData = RobotData(items)

  otherBin = deepcopy(robotData)
  otherBin.removeItem(targetItem)

  calculateGraspScore(grasps[0], items[0], otherBin)
