import numpy as np
import cv2 as cv

cam = cv.VideoCapture(0)

fourcc = cv.VideoWriter_fourcc(*'MP4V')
out = cv.VideoWriter('output.mp4',fourcc,30.0,(640,480))

if not cam.isOpened():
	print ("Open Camera")
	exit()

while cam.isOpened():

	ret,frame = cam.read()
	
	if not ret:
		print("Can't read frame")
		break
	
	out.write(frame)

	cv.imshow('Webcam',frame)

	if cv.waitKey(1) == ord('q'):
		break

cam.release()
out.release()
cv.destroyWindow('Webcam')

play = cv.VideoCapture('output.mp4')

if not play.isOpened():
        print ("Camera")
        exit()


while play.isOpened():

	boo, image = play.read()
	
	if not boo:
		print("Can't read frame, NOOOO")
		break

	cv.imshow('Webcam',image)
	cv.waitKey(30)
	if cv.waitKey(1) == ord('q'):
		break

play.release()
cv.destroyAllWindows()
