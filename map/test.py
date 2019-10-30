import cv2

img = cv2.imread("stage4.pgm")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("map", gray)
cv2.waitKey(0)