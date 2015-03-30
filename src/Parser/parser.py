import json
import cv2
import libApcRobot

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
    row = position % 3
    column = int(position/3)
    robotData.setWorkOrder(row,column,order)
