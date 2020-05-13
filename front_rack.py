import cv2
import rosbag
from cv_bridge import CvBridge
import numpy as np
import math

bag = input("Enter the name of the rosbag file:")
bag = rosbag.Bag(bag, "r")
bridge = CvBridge()

winName = 'Blue Target Detect'
winName2 = "video capture"

for topic, msg, t in bag.read_messages(topics=["mono_front/usb_cam/image_rect_color"]):
	image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
	
	height, width, layers = image.shape
	centre = (int(width/2), int(height/2))
	store = []
	count = 1

	hsv_frame = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

	#creating double masks and adding them together to ensure that the rack is clearly distinguished from environment
	#lower blue mask
	lower_blue = np.array([30,158,124])
	upper_blue = np.array([145,255,255])
	lower_blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
	#upper blue mask
	lower_blue = np.array([120,100,84])
	upper_blue = np.array([200,255,255])
	upper_blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

	blue_mask = lower_blue_mask + upper_blue_mask
	blue_image= cv2.bitwise_and(image, image, mask=blue_mask)
	blue_image = cv2.GaussianBlur(blue_image,(7,7),cv2.BORDER_DEFAULT) 
	drawnImage = blue_image

	#converting to grayscale image for contour detecting
	gray = cv2.cvtColor(blue_image, cv2.COLOR_BGR2GRAY)
	ret, threshImg = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
	

    # find the contours in the thresholded image...
	_,contours, _ = cv2.findContours(threshImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


	for c in contours:
		if cv2.contourArea(c) < 4000:
			continue
		(x,y,w,h)= cv2.boundingRect(c)

		#To detect only bounding rectangles with greater height from width and storing the x and y coordinates of the points in the bunding rectangle in a list
		if h>w:
			cv2.rectangle(drawnImage,(x,y),(x+w,y+h),(255,0,0),2)
			Centroid = (int((x + x + w) /2), int((y + y + h) /2))
			cv2.circle(drawnImage, Centroid, 1, (54, 255, 164), 4)
			cv2.circle(drawnImage, centre, 1, (115, 0, 255), 4)
			cv2.putText(drawnImage,"image centre",centre,cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
			cv2.putText(drawnImage,"point",Centroid,cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
			dist_from_cent = [centre[0]-Centroid[0], centre[1]-Centroid[1]]
			store.append([x,y]) 
			
			#ensuring that the robot is in the centre within 20% of the centre of the image
			if len(store)>1:
				print(store)
				for i in range(len(store)):
					while count < len(store):
						dist_between = abs(store[i][0]-store[count][0])
						print("dist_between points =" , dist_between)
						if dist_between >= 700:
							dist_cent1 = abs(store[i][0] - centre[0])
							dist_cent2 = abs(store[count][0] - centre[0])
							if dist_cent2<= 1.2*dist_cent1 or  dist_cent1<= 1.2*dist_cent2:
								print("Front rack detected...")
						count+=1
							
	
	cv2.imshow(winName, drawnImage)
	cv2.imshow(winName2, image)

	# get the key from the keyboard
	key = cv2.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

cv2.destroyAllWindows()