import sys
import string
import json
import cv2
import apcRobot
import pyAPC

if __name__ != "__main__":
    print("can't import me")

with open("data/example.json") as goalsFile:
    goals = json.load(goalsFile)

robotData = apcRobot.RobotData.getInstance()

binContents = goals['bin_contents']
workOrders = goals['work_order']

binNumber=12
binNames=map(lambda x:'bin_'+string.ascii_uppercase[x], range(0,binNumber))
print binNames


print binContents
for position in range(len(binNames)):
    column = position % 3
    row = int(position/3)
    binName = binNames[position]
    itemNames = binContents[binName]
    for index in range(len(itemNames)):
        robotData.setBinItem(row,column,index,str(itemNames[index]))

for position in range(len(binNames)):
    column = position % 3
    row = int(position/3)
    for index in range(5):
        itemName = robotData.getBinItem(row,column,index)
        print(row,column,index,itemName)

for order in workOrders:
    position=binNames.index(order['bin'])
    column = position % 3
    row = int(position/3)
    print(str(order['item']))
    print("I'm flaggin"+str(order['item'])+" into row"+str(row)+" column"+str(column))


    robotData.setWorkOrder(row,column,str(order['item']))

pyAPC.theBigAPCPythonMain(len(sys.argv), sys.argv);
