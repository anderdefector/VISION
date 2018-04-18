#!/usr/bin/env python
#from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
import vlidar as vl
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber("/bebop/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    delta = 60
    rows,cols,ch = cv_image1.shape  
    cv_image = cv_image1[(rows/2)-delta:(rows/2)+delta, ::]
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0,120,120]), np.array([255,255,255]))
    
    if cv2.countNonZero(mask) > 500: 
      #delta = 30
      #rows,cols,ch = mask.shape
      #smallerMask = mask[(rows/2)-delta:(rows/2)+delta, ::]

      img,lidar = vl.Main_Postes(mask.copy(), 80, 396.17782, 0.10, 320, 184)
      cv2.imshow('video',img)
      if len(lidar) >=2:
	av_rho, av_theta = Average(lidar)
'''
				g,h,rho = CalcMovement(lidar)
				Anglerror(lidar)
				try:
					print ('g= '+ str(g) +' h='+str(h)+' rho='+str(rho))
					print (lidar)
				except(NameError, RuntimeError):
					pass
'''

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def CalcMovement(lidar):
	c = abs(max(lidar[0][0],lidar[1][0]))		#calculate which distance is the biggest
	b = abs(min(lidar[0][0],lidar[1][0]))		#calculate which distance is the smallest

	if max(lidar[0][0],lidar[1][0]) == lidar[1][0]:
		theta1 = abs(lidar[1][1])
		theta2 = abs(lidar[0][1])
	else:
		theta1 = abs(lidar[0][1])
		theta2 = abs(lidar[1][1])

		#The trigonometric functions from math needs an input in radians
	alpha = abs(theta1) + abs(theta2)		#add the two absolute values of the angles
	a = (c**2 + b**2 - 2*b*c*math.cos(math.radians(alpha)))**(0.5)			#calculate the missing side of the triangle
	beta = math.asin(b*(math.sin(math.radians(alpha)))/a)					#get the value of beta in radians
	gamma = 180 - math.degrees(beta) - alpha
	#e = gamma-beta
	#print ("Error = ",e)
	phi1 = 180-abs(theta1)-math.degrees(beta)
	d = (c * math.sin((beta)))/math.sin(math.radians(phi1))
	
	ksi = (phi1) - 90
	rho = 180 - phi1 - theta1
	h = d*math.sin(math.radians(ksi))
	g = (d**2 - h**2)**(0.5)
	if theta1 < 0:
		g = -g
	return g,h,rho

def Average(lidar):
	av_rho = (lidar[0][0] + lidar[1][0])/2
	av_theta = (lidar[0][1] + lidar[1][1])/2
	return av_rho, av_theta
	

	


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
