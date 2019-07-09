import numpy as np
import cv2 as cv
cap = cv.VideoCapture('output.avi')
while cap.isOpened():
    ret, frame = cap.read()
    cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.imshow('frame', gray)
    cv.waitKey(25)
    if cv.waitKey(1) == ord('q'):
        break
cap.release()
cv.destroyAllWindows()
