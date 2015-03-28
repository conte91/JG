import RobotData as robotData
import numpy as np
from csg.core import CSG


class Item(object):
    def __init__(self, pose, roughShape, shape, grasps):
        self.pose = pose
        self.roughShape = roughShape
        self.shape = shape
        self.grasps = grasps

    def translate(self, translationMatrix):
        self.pose += translationMatrix
        self.roughShape += translationMatrix


class Grasp(object):
    def __init__(self, approachPose, gripPose, minForce, maxForce):
        self.approachPose = approachPose
        self.gripPose = gripPose
        self.minForce = minForce
        self.maxForce = maxForce

    def translate(self, translationMatrix):
        self.approachPose += translationMatrix
        self.gripPose += translationMatrix

    def checkCollision(self, otherItem):
        # missing shape <-> shape collision checking
        roughShape = CSG.cylinder(radius=1)
        if roughShape.intersect(otherItem.roughShape):
            return True
        return False

"""
    python::class_<InterProcessCommunication::RobotData, boost::noncopyable>("RobotData", python::no_init)
      .def("getInstance",&InterProcessCommunication::RobotData::getInstance,python::return_value_policy<python::reference_existing_object>() )
      .staticmethod("getInstance")
      .def("getBinItem",&InterProcessCommunication::RobotData::getBinItem)
      .def("setBinItem",&InterProcessCommunication::RobotData::setBinItem)
      .def("getImageFrame",&InterProcessCommunication::RobotData::getImageFrame)
      .def("createGrasp",&InterProcessCommunication::RobotData::getImageFrame)
      ;
"""


def getBestGrasp(targetItem):
    binItems = robotData.getBinItems()
    binItems.pop(targetItem)

    translationMatrix = robotData.getItemPose(targetItem)
    targetItem.translate(translationMatrix)

    score = []
    for grasp in targetItem.getGrasps():
        score[grasp] = calculateGraspScore(grasp)


def calculateGraspScore(grasp, targetItem, otherItems):
    if not isDoable(grasp, targetItem):
        return -1

    for binItem in binItems:
        score[grasp] += grasp.checkCollision(binItem)


def isDoable(itemName, grasp, row, column):
    global gripper
    translationMatrix = robotData.bins[row, column].getObjectPose(itemName)
    shape = approx[itemName]
    # get polygons of the real object
    itemPolygons = shape + translationMatrix
    # get the gripper translated by grip.apporach
    translationMatrix = generateTranslationMatrix(grasp['approach'])
    gripperPolygons = gripper + translationMatrix
    # draw gripper on polygon
    movingArea = gripperPolygons.union(itemPolygons)
    otherItems = robotData.getItems()
    for otherItem in getItem(Bin):
        if item, itemPose > 0:
            return False
    return True


if __name__ == '__main__':
    grasps = {}
    gripper = "somegripper"
    originPose = {'x': 0, 'y': 0, 'z': 0, 'alpha': 0, 'beta': 0, 'gamma': 0, }

    dummyGrasp = Grasp(originPose, originPose, 0, 100000)
    dummyShape = CSG.cylinder(radius=1)
    dummyPose = originPose

    items = []
    for i in range(12):
        grasps = []
        for j in range(4):
            grasps.append(dummyGrasp)
        items.append(Item(originPose,dummyShape,dummyShape,grasps))
