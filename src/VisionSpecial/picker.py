import cv2
import numpy as np
from time import sleep
 
colors = []
 
def on_mouse_click (event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONUP:
        colors.append(frame[y,x].tolist())
 
def main():
    #capture = cv2.VideoCapture(0)
 
    frame = cv2.imread("balls.jpg")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    while True:
        #_, frame = capture.read()
        if colors:
            print colors[-1]
        cv2.imshow('frame', hsv)
        cv2.setMouseCallback('frame', on_mouse_click, hsv)
 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 
    cv2.destroyAllWindows()
 
    # avgb = int(sum(c[0] for c in colors) / len(colors))
    # avgg = int(sum(c[0] for c in colors) / len(colors))
    # avgr = int(sum(c[0] for c in colors) / len(colors))
    # print avgb, avgg, avgr
 
    minb = min(c[0] for c in colors)
    ming = min(c[1] for c in colors)
    minr = min(c[2] for c in colors)
    maxb = max(c[0] for c in colors)
    maxg = max(c[1] for c in colors)
    maxr = max(c[2] for c in colors)
    print minr, ming, minb, maxr, maxg, maxb
 
    lb = [minb,ming,minr]
    ub = [maxb,maxg,maxr]
    print lb, ub
 
if __name__ == "__main__":
    main()
