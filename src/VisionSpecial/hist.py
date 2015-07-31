img = cv2.imread("blueball.jpg")
color = ('b','g','r')
points = {}
for i,col in enumerate(color):
    histr = cv2.calcHist([img],[i],None,[256],[0,256])
    plt.plot(histr,color = col)
    plt.xlim([0,256])
    points[col] = histr.index(max(histr))
plt.show()
