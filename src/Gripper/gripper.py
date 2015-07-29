from copy import deepcopy
import transformations as tr
import numpy as np
# import math

GRASP_ERROR_LIMIT = 1000
VERY_NEGATIVE_NUMBER = -313373


def main():
    global originPose
    global originXYZ
    global robotData

    originPose = tuple([0]*6)
    originXYZ = (0, 0, 0)
    # end globals

    targetItem = 'mead_index_cards'  # sys.argv[1]
    robotData = RobotData()
    bestGrasp = getBestGrasp(targetItem)
    print "best score for {} is: {}".format(targetItem, bestGrasp)
    with open("/tmp/grasp.result", "w") as output:
        output.write(repr(bestGrasp))

def changeReference(point, newReference):
    p0 = list(point) + [0]

    x0, y0, z0, a, b, g = newReference
    rotMatrix = tr.euler_matrix(a, b, g)

    newPose = np.dot(rotMatrix, p0) + [x0, y0, z0, 0]
    return newPose[:-1]

def distance(p0, p1):
    dimensions = len(p0)
    squares = 0
    for i in range(dimensions):
        squares += (p0[i] - p1[i]) ** 2
    return pow(squares, 1/dimensions)


class Shape(object):
    def __init__(self, elements):
        self.elements = elements

    def __repr__(self):
        return 'Shape(\n  ' + '\n  '.join(
            [repr(element) for element in self.elements]
        ) + '\n)'

    def intersect(self, otherShape):
        total = 0
        for e0 in self.elements:
            for e1 in otherShape.elements:
                total += e0.getCollisionScore(e1)
        return total

    def changeReference(self, itemPose):
        for i in range(len(self.elements)):
            self.elements[i].changeReference(itemPose)


def project(point, vector):
    print "TODO: floating point operations fuckups"
    x, y, z = range(3)
    n = d = 0.
    result = 0.

    for axis in [x, y]:  # z]:
        n += point[axis] * vector[axis]
        d += vector[axis]**2
    # edit vector inplace
    vector = list(vector)
    result = tuple([float(el) * (n/d) for el in vector])
    return result


def intersectCuboidCuboid(cube0, cube1):
    x, y, z = range(3)
    print "intersectCuboidCuboid should return a value"

    # calculate vertex (lower upper right left)
    ur0 = cube0[1]
    ul0 = (cube0[0][x], cube0[1][y])
    lr0 = cube0[0]
    # ll0 = (cube0[1][x], cube0[0][y])
    ur0 = cube0[1]

    ur1 = cube1[1]
    ul1 = (cube1[0][x], cube1[1][y])
    lr1 = cube1[0]
    # ll1 = (cube1[1][x], cube1[0][y])
    ur1 = cube1[1]
    # calculate axes
    axis00 = ur0 - ul0
    axis01 = ur0 - lr0
    axis10 = ur1 - ul1
    axis11 = ur1 - lr1

    for axis in [axis00, axis01, axis10, axis11]:
        projectedPoints0 = [project(vertex, axis) for vertex in cube0]
        projectedPoints1 = [project(vertex, axis) for vertex in cube1]

        minCube0 = min(projectedPoints0)
        maxCube0 = max(projectedPoints0)
        minCube1 = min(projectedPoints1)
        maxCube1 = max(projectedPoints1)
        if minCube0 < maxCube1 and minCube1 < maxCube0:
            # there is an intersection on this axis
            pass
        else:
            # there is a plane separating the cubes
            return False
    return True


class PrimitiveShape(object):
    def __init__(self):
        print "primitive shape can only be extended"
        raise Exception("can't init PrimitiveShape")

    def getCollisionScore(self, otherShape):
        name0 = type(self).__name__
        name1 = type(otherShape).__name__
        names = sorted((name0, name1))

        if names == ["Cuboid", "Cuboid"]:
            thisCuboid = self.vertices
            otherCuboid = otherShape.vertices
            overlap = 0
            x, y, z = range(3)

            x0 = min(thisCuboid[0][x], otherCuboid[0][x])
            y0 = min(thisCuboid[0][y], otherCuboid[0][y])
            z0 = min(thisCuboid[0][z], otherCuboid[0][z])

            x1 = max(thisCuboid[0][x], otherCuboid[0][x])
            y1 = max(thisCuboid[0][y], otherCuboid[0][y])
            z1 = max(thisCuboid[0][z], otherCuboid[0][z])

            a0 = min(thisCuboid[1][x], otherCuboid[1][x])
            b0 = min(thisCuboid[1][y], otherCuboid[1][y])
            c0 = min(thisCuboid[1][z], otherCuboid[1][z])

            a1 = max(thisCuboid[1][x], otherCuboid[1][x])
            b1 = max(thisCuboid[1][y], otherCuboid[1][y])
            c1 = max(thisCuboid[1][z], otherCuboid[1][z])

            x = max(max(a0, x0)-min(a1, x1), 0)
            y = max(max(b0, y0)-min(b1, y1), 0)
            z = max(max(c0, z0)-min(c1, z1), 0)

            overlap = pow(x*y*z, 1./3)
            return overlap

        elif names == ["Cuboid", "Sphere"]:

            (v1, v2) = self.vertices
            (c, r) = (otherShape.center, otherShape.radius)

            # assume C1 and C2 are element-wise sorted, if not, do that now

            if c[0] < v1[0]:
                # the sphere center is left of cube
                r -= abs(c[0] - v1[0])
            elif c[0] > v2[0]:
                # the sphere center is right of the cube
                r -= abs(c[0] - v2[0])
            # if it's inside, do nothing

            if c[1] < v1[1]:
                r -= abs(c[1] - v1[1])
            elif c[1] > v2[1]:
                r -= abs(c[1] - v2[1])

            if c[2] < v1[3]:
                r -= abs(c[2] - v1[3])
            elif c[2] > v2[3]:
                r -= abs(c[2] - v2[3])
            # r = r - |dx| + |dy| + |dz|
            return max(r, 0)

        elif names == ["Sphere", "Sphere"]:

            c0 = self.center
            r0 = self.radius
            c1 = otherShape.center
            r1 = otherShape.radius

            coveredSpace = r0 + r1
            spaceBetween = distance(c0, c1)
            intersectionLen = spaceBetween - coveredSpace
            return max(0, intersectionLen)


class Cuboid(PrimitiveShape):
    def __init__(self, v0, v1):
        tv0 = v0
        tv1 = v1
        v1 = (
            min(tv0[0], tv1[0]),
            min(tv0[1], tv1[1]),
            min(tv0[2], tv1[2])
            )
        v1 = (
            max(tv0[0], tv1[0]),
            max(tv0[1], tv1[1]),
            max(tv0[2], tv1[2])
            )
        self.vertices = (v0, v1)

    def changeReference(self, itemPose):
        v0 = changeReference(self.vertices[0], itemPose)
        v1 = changeReference(self.vertices[1], itemPose)
        self.vertices = (v0, v1)

    def __repr__(self):
        return 'Cuboid(' + ' '.join([
            repr(vertex) for vertex in self.vertices
        ]) + ')'

    def __name__(self):
        return 'Cuboid'


class Sphere(PrimitiveShape):
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
        self.shapeName = "sphere"

    def __repr__(self):
        return "Sphere(" + self.center + ", " + self.radius + ")"


class RobotData(object):
    def __init__(self):
        itemsDatabase = dict()

        pickableObjects = {
            'munchkin_white_hot_duck_bath_toy': (150, 160, 80),
            'mark_twain_huckleberry_finn': (128, 250, 16),
            'sharpie_accent_tank_style_highlighters': (120, 130, 20),
            'genuine_joe_plastic_stir_sticks': (144, 97, 108),
            'safety_works_safety_glasses': (220, 55, 115),
            'rollodex_mesh_collection_jumbo_pencil_cup':  (100, 136, 100),
            'dr_browns_bottle_brush': (350, 140, 50),
            'kygen_squeakin_eggs_plush_puppies': (240, 60, 130),
            'kong_duck_dog_toy': (170, 110, 70),
            'mommys_helper_outlet_plugs': (190, 60, 90),
            'highland_6539_self_stick_notes': (150, 40, 50),
            'expo_dry_erase_board_eraser': (130, 35, 55),
            'paper_mate_12_count_mirado_black_warrior': (195, 20, 50),
            'laugh_out_loud_joke_book': (180, 10, 110),
            'stanley_66_052': (195, 20, 100),
            'mead_index_cards': (130, 78, 23),
            'elmers_washable_no_run_school_glue': (65, 150, 35),
        }
        baseGraspParams = { 'approach':(0, 0, 0), 'grip':(0, 0, 0), 'min':1, 'max':10 }
        graspsOf = {
            'kygen_squeakin_eggs_plush_puppies': [baseGraspParams,],
            'sharpie_accent_tank_style_highlighters': [baseGraspParams,],
            'expo_dry_erase_board_eraser': [baseGraspParams,],
            'munchkin_white_hot_duck_bath_toy': [baseGraspParams,],
            'mark_twain_huckleberry_finn': [baseGraspParams,],
            'genuine_joe_plastic_stir_sticks': [baseGraspParams,],
            'safety_works_safety_glasses': [baseGraspParams,],
            'rollodex_mesh_collection_jumbo_pencil_cup': [baseGraspParams,],
            'dr_browns_bottle_brush': [baseGraspParams,],
            'kong_duck_dog_toy': [baseGraspParams,],
            'mommys_helper_outlet_plugs': [baseGraspParams,],
            'highland_6539_self_stick_notes': [baseGraspParams,],
            'paper_mate_12_count_mirado_black_warrior': [baseGraspParams,],
            'laugh_out_loud_joke_book': [baseGraspParams,],
            'stanley_66_052': [baseGraspParams,],
            'mead_index_cards': [baseGraspParams,],
            'elmers_washable_no_run_school_glue': [baseGraspParams,],
        }
        x, y, z =  range(3)
        # update graspPoses to avoid crushing the object
        for name,data in graspsOf.iteritems():
            for i in range(len(data)):
                x1 = data[i]['approach'][x]
                y1 = -pickableObjects[name][y]
                z1 = data[i]['approach'][z]
                graspsOf[name][i]['approach']  = (x1, y1, z1)
                x1 = data[i]['grip'][x]
                y1 = -pickableObjects[name][y]/2
                z1 = data[i]['grip'][z]
                graspsOf[name][i]['grip']  = (x1, y1, z1)

        print "fineShapes are fake"
        fakeCuboids = [
            Cuboid(originXYZ, (1, 2, 1)),
            Cuboid(originXYZ, (2, 2, 3)),
        ]

        for objName in pickableObjects.keys():
            roughElements = [Cuboid(originXYZ, pickableObjects[objName])]
            roughShape = Shape(roughElements)
            fineShape = Shape(fakeCuboids)

            grasps = []
            for grasp in graspsOf[objName]:
                approachPose = grasp['approach']
                gripPose = grasp['grip']
                minForce = grasp['min']
                maxForce = grasp['max']
                grasps.append(Grasp(approachPose, gripPose, minForce, maxForce))

            itemsDatabase[objName] = Item(objName, originPose, roughShape, fineShape, grasps)
        self.itemsDatabase = itemsDatabase

    def __repr__(self):
        return '\n'.join([
            repr(item) for item in self.itemDatabase.values()[:5]
        ])+'...'

    def getBin(self):
        global robotData
        with open("/tmp/robot.data") as dataFile:
            binItems = dataFile.read().split("\n")[:-1]
        binItems = [item.split(" ") for item in binItems]
        result = {}
        for binItem in binItems:
            item = robotData.itemsDatabase[binItem[0]]
            item.setPose(tuple(map(float,binItem[1:])))
            result[binItem[0]] = item
        return KivaBin(result)

    def getItemTemplate(self, itemName):
        return self.itemsDatabase[itemName]

    def removeItem(self, item):
        raise Exception("can't remove item from the database!")
        ind = self.itemDatabase.index(item)
        self.itemDatabase.pop(ind)


class KivaBin(object):
    # fake
    def __init__(self, items):
        self.items = items

    def __repr__(self):
        return "KivaBin(\n\t" + '\n\t'.join([
            repr(item) for item in self.items
        ]) + "\n)"

    def getBinItems(self):
        return list(self.items.values())

    def removeItem(self, item):
        itemName = item._name
        if not self.items.has_key(itemName):
            raise Exception("""trying to remove a non existing
                            item from shelf model""")
        del self.items[itemName]

    def getItemPose(self, itemName):
        return self.items[itemName]._pose


class Item(object):
    def __init__(self, name, pose, roughShape, fineShape, grasps):
        self._name = name
        self._pose = pose
        self._roughShape = roughShape
        self._fineShape = fineShape
        self._grasps = grasps

    def __hash__(self):
        return hash(hash(self._name) + sum(self._pose))

    def __repr__(self):
        return "Item(" + self._name + " at " + repr(self._pose) + ")"

    def __eq__(self, other):
        return (self._name, self._pose) == (other._name, other._pose)

    def setPose(self, newPose):
        self._pose = newPose

    def getGrasps(self):
        grasps = []
        for grasp in self._grasps:
            point = list(grasp.gripPose)
            grasp.gripPose = changeReference(point, self._pose)
            grasps.append(grasp)
        return grasps


class Grasp(object):
    def __init__(self, approachPose, gripPose, minForce, maxForce):
        self.approachPose = approachPose
        self.gripPose = gripPose

        self.roughShape = Shape([
            Cuboid((16, 0, 0), (27, 50, 28)),
            Cuboid((0, 15, 0), (36, 30, 28)),
        ])
        self.minForce = minForce
        self.maxForce = maxForce

    def __repr__(self):
        return "Grasp(" + ' '.join([
            str(val) for val in self.gripPose
        ]) + ")"

    def getCollisionVolume(self, otherItem):
        shape = deepcopy(self.roughShape)
        shape.changeReference(otherItem._pose)
        volume = shape.intersect(otherItem._roughShape)
        return volume


def getBestGrasp(targetItemName):
    targetBin = robotData.getBin()
    # load the item from shapes db
    targetItem = robotData.getItemTemplate(targetItemName)
    itemPose = targetBin.getItemPose(targetItemName)
    targetItem.setPose(itemPose)

    if min(itemPose) < -1000:
        # -1000 as pose means item not found
        return VERY_NEGATIVE_NUMBER

    # adapt the item template to the real item
    grasps = targetItem.getGrasps()

    # force best graps to be None if <0
    grasps.insert(0, None)
    bestGrasp = max(grasps, key=lambda x:
                    calculateGraspScore(x, targetItem, targetBin))
    return bestGrasp


def calculateGraspScore(grasp, targetItem, targetBin):
    if grasp is None:
        return -0
    scores = dict()
    leftoverBin = deepcopy(targetBin)
    leftoverBin.removeItem(targetItem)

    #if not isDoable(grasp, targetItem):
    #    return -1

    collisionVolume = 1
    for binItem in leftoverBin.getBinItems():
        scores[binItem] = grasp.getCollisionVolume(binItem)
        collisionVolume += scores[binItem]

    bestScore = 1.0
    if collisionVolume == 0:
        score = bestScore
    else:
        score = bestScore/collisionVolume
    print score
    return score


def isDoable(grasp, targetItem):
    """
    Checks if the gripper can grasp the item ignoring the environment
    we only allow GRASP_ERROR_LIMIT error
    """
    global GRASP_ERROR_LIMIT
    retr = True
    runningSum = 0
    grip = grasp.gripPose
    for i in range(len(grasp.gripPose)):
        runningSum += (grasp.gripPose[i] - targetItem._pose[i]) ** 2
    grasp_error = np.sqrt(runningSum)
    if grasp_error > GRASP_ERROR_LIMIT:
        retr = False
    return retr

if __name__ == '__main__':
    main()
else:
    print "u no run me"
