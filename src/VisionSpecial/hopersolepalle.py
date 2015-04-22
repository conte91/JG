import itertools
import numpy as np
import cv2
import sys
from copy import deepcopy

#img = cv2.imread("/tmp/myballs.png")
img = cv2.imread("notballs.png")
#img = cv2.imread("balls.png")
#img = cv2.resize(img,(353,503)) #640*480

img = cv2.blur(img,(5,5))
#doesn't do much
for i in range(img.shape[2]):
  img[i] = cv2.equalizeHist(img[i])

imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

#doesn't do much
for i in range(img.shape[2]):
  imgHSV[i] = cv2.equalizeHist(imgHSV[i])

imgBGR = img.copy()
imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

drawing = img.copy()
mask = np.zeros_like(imgGray) 

triangle = list()
circles = list()
candidates = dict()

#hue is 1-180
colors = {
    'blue':{
      'hsv':((95,15,15),(105,255,255)),
      'bgr':((255,212,0),(255,127,0)),
    },
    'yellow':{
      'hsv':((15,100,50),(25,255,255)),
      'bgr':((0,127,255),(0,212,255)),
    },
    'orange':{
      'hsv':((9,104,192),(15,190,255)),
      'bgr':((0,59,255),(0,119,255)),
      },
  }

for colorName,colorValues in colors.iteritems():

  candidates[colorName] = []

  colorMin = np.array(colorValues['hsv'][0],np.uint8)
  colorMax = np.array(colorValues['hsv'][1],np.uint8)
  match = cv2.inRange(imgHSV,colorMin,colorMax)

  contours,hierarchy = cv2.findContours(match,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  
  if not contours:
    print "ERROR contours not found for hsv:",colorValues['hsv']
    continue

  for contour in contours:
    M = cv2.moments(contour)
    area = M['m00']
    if area > 100:
      print area
      center,radius = cv2.minEnclosingCircle(contour)
      center = (int(center[0]),int(center[1]))
      x = int(M['m10'] / area)
      y = int(M['m01'] / area)
      triangle.append((x,y))

      error = 0
      p = center
      for q in contour:
        q = q[0]
        #q = q.transpose()
        distance = np.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)
        derr = radius - distance
        error += derr ** 2

      maxError = ( ( radius * len(contour)) ** 2 ) * 0.35 
      cv2.drawContours(drawing,[contour],0,colorValues['bgr'][0],thickness=1)
      if error < maxError:
        cv2.drawContours(drawing,[contour],0,colorValues['bgr'][0],thickness=2)
        print center,radius,"is good", error, "<", maxError
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.circle(drawing,center,int(radius),colorValues['bgr'][1],thickness=2)
        cv2.putText(drawing,str(int(error)),center, font, 0.5,(255,255,255),1)
        x,y = (center[0],center[1]-15)
        cv2.putText(drawing,str(colorName),(x,y), font, 0.5,(255,255,255),1)
        circles.append((center,radius,error))

      #cv2.putText(drawing,error,cv2.FONT_HERSHEY_PLAIN,1,(0,0,0))



#find White
_,match = cv2.threshold(imgGray,250,255,0)
if 0 == match.max():
  print "ERROR white not found"

#draw white points on grayscale
mask = cv2.bitwise_not(mask)
imgGray = cv2.bitwise_and(imgGray,mask)


#hist = cv2.calcHist([imgBGR],[range(3)],None,[9],(0,255))
#hist2 = cv2.normalize(hist,hist,0,255,cv2.NORM_MINMA)

"""

circles = dict()
for color,candidates in candidates.iteritems():
  circles[color] = list()
  for i in range(len(candidates)):
    circle = cv2.minEnclosingCircle(candidates[i])
    circles[color].append(circle)

def findSimilarCircle(blueCircle):
  d1 = map(lambda yellowCircle:(yellowCircle[1]-blueCircle[1])**2,circles['yellow'])
  d2 = map(lambda orangeCircle:(orangeCircle[1]-blueCircle[1])**2,circles['orange'])
  print blueCircle,d1,d2
  return d1,d2

d0 = map(findSimilarCircle,circles['blue'])

"""


if len(triangle) != 3:
  cv2.imshow("result_contours",drawing)
  print "fail"
  cv2.waitKey()
  cv2.destroyAllWindows()
  sys.exit()

a,b,c = triangle
grippableBall = min(triangle, key=lambda x:x[1])

with open("/tmp/grasp.pose","w") as gp:
  gp.write(repr(grippableBall + (0,0,0,0)))

area = (a[0]*(b[1]-c[1])+b[0]*(c[1]-a[1])+c[0]*(a[1]-b[1]))/2

print "TODO normalize for bin size" #or ball size

if area > 10000:
  print "ERROR: area is too big"
if area < 2500:
  print "found points are too near"

for (p,q) in itertools.combinations(triangle,2):
  distance = np.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)
  cv2.line(drawing,p,q,(0,0,255),5)

  #cv2.circle(img2,(x,y), 1, (255,255,0) ,10)
  #mask = cv2.bitwise_or(mask,match)

#result = cv2.bitwise_and(imgGray,mask)

#cv2.imshow("result",result)
#cv2.waitKey()




#cv2.drawContours(img2,contours,-1,(0,0,255),thickness=2)
cv2.imshow("result_contours",drawing)
cv2.waitKey()
cv2.destroyAllWindows()
