import RobotData as robotData
import numpy as np
from csg.core import CSG


GRASP_ERROR_LIMIT = 1


class Item(object):
    def __init__(self, pose, roughShape, shape, grasps):
        self.pose = pose
        self.roughShape = roughShape
        self.shape = shape
        self.grasps = grasps

    def __hash__(self):
        return hash((self.pose ,self.shape))

    def __eq__(self, other):
        return (self.pose, self.shape) == (other.pose, other.shape)

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

    def getCollisionVolume(self, otherItem):
        # missing shape <-> shape collision checking
        roughShape = CSG.cylinder(radius=1)
        volume = roughShape.intersect(otherItem.roughShape)
        return volume

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
        score[grasp] = calculateGraspScore(grasp,)


def calculateGraspScore(grasp, targetItem, bin):
    if not isDoable(grasp, targetItem):
        return -1

    scores = dict()
    for binItem in bin.getItems():
        collisionVolume = 0
        for grasp in targetItem.getGrasps():
            collisionVolume += grasp.checkCollision(binItem)
        scores[binItem] = collisionVolume
    return scores


def isDoable(grasp, targetItem):
    """
    Checks if the gripper can grasp the item ignoring the environment
    we only allow GRASP_ERROR_LIMIT error
    """
    distance = np.sqrt(np.square(grasp.gripPose) + np.square(targetItem.pose))
    if distance > GRASP_ERROR_LIMIT:
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
        someItem = Item(originPose, dummyShape, dummyShape, grasps)
        items.append(someItem)
