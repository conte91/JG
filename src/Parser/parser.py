import json
import cv2
import libApcRobot

class RobotController(object):
    def __init__(self,robotData,graspInfo):
        self.robotData = robotData
        self.graspPosesScores = graspInfo['graspPosesScores']

    def goToBin(self,binName):
        binNumber = 'A' - binName[4]
        print("moving to bin ",binNumber)

    def findBestGraph(graspPoseLeft):
        best = max(self.graspPosesScores[graspPoseLeft])
        score = self.graspPosesScores[best]
        return best,score


    def findCollisions(bodies,graspPoses):
        return

    def calculateScore(visionFrame,graspInfo):
        bigNumber = 1000
        graspPosesLeft = graspInfo['graspPosesLeft']
        results = {}
        for graspPoseLeft in graspPosesLeft:
            best,score = findBestGrasp(graspPoseLeft)
            results[graspPoseLeft] = (score,best)



    def pickItem(self,itemName):
        visionFrame = robotData.getLastFrame()
        (xyz,rpy) = getPose(visionFrame)
        grippingInfos = getGrippinPositions(itemName)
        for grippingInfo in grippingInfos:
            calculateScore(vsionFrame,grippingInfo)


if __name__ != "__main__":
    print("can't import me")

with open("data/example.json") as goalsFile:
    goals = json.load(goalsFile)

robotData = apcRobot.RobotData()

binContents = goals['bin_contents']
workOrders = goals['work_order']

binNames = sorted(binContents.keys())

for position in range(len(binNames)):
    row = position % 3
    column = int(position/3)
    binName = binNames[position]
    itemNames = binContents[binName]
    for index in range(len(itemNames)):
        robotData.setBinItem(row,column,index,str(itemNames[index]))

for position in range(len(binNames)):
    row = position % 3
    column = int(position/3)
    for index in range(5):
        itemName = robotData.getBinItem(row,column,index)
        print(row,column,index,itemName)

for order in workOrders:
    shelfBin = order['bin']
    targetItem = order['item']
    robotController.goToBin(shelfBin)
    robotController.pickItem(targetItem)
