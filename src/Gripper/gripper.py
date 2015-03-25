import RobotData as rd
from csg.core import CSG

def getBestGrasp(itemName,row,column):
    global grasps
    for grasp in grasps:
        if(isDoable(itemName,grasp)):
            return grasp
    return None

#check if the end effector touches other stuff
def isDoable(itemName,grasp):
    global gripper
    translationMatrix = robotData.bins[row,column].getObjectPose(itemName)
    shape = approx[itemName]
    #get polygons of the real object
    itemPolygons = translate(shape,translationMatrix)
    #get the gripper translated by grip.apporach
    translationMatrix = generateTranslationMatrix(grasp['approach'])
    gripperPolygons = translate(gripper,translationMatrix)
    #draw gripper on polygon
    movingArea =  gripperPolygons.union(itemPolygons)
    for item in Bin:
        if intersect(item,itemPose) > 0:
            return False
    return True

if __name__ == '__main__':
    grasps = {}
    OriginPose = { 'x':0, 'y':0, 'z':0, 'alpha':0, 'beta':0, 'gamma':0, }
    graspSafe = OriginPose
    graspNear = OriginPose
    forceMin = 0
    forceMax = 999
    graspFuffa = {'poseSafe':graspSafe, 'poseNear':graspNear,'forceMin':forceMin,'forceMax':forceMax,}

    grasps['obj0'] = [graspFuffa,]
    grasps['obj1'] = [graspFuffa,graspFuffa,]
    grasps['obj2'] = [graspFuffa,graspFuffa,graspFuffa,]
    grasps['obj3'] = [graspFuffa,graspFuffa,graspFuffa,graspFuffa,]
    grasps['obj4'] = [graspFuffa,graspFuffa,graspFuffa,]
    grasps['obj5'] = [graspFuffa,graspFuffa,]
    grasps['obj6'] = [graspFuffa,]
    grasps['obj7'] = [graspFuffa,graspFuffa,]

    approx['obj0'] = [1,1,0]
    approx['obj1'] = [1,1,1]
    approx['obj2'] = [1,1,2]
    approx['obj3'] = [1,1,3]
    approx['obj4'] = [1,1,4]
    approx['obj5'] = [1,1,5]
    approx['obj6'] = [1,1,6]
    approx['obj7'] = [1,1,7]
