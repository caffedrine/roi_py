#!/usr/bin/env python
#Subscribe and Publish at the same time

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest

from roi.msg import sendMsg
from roi.msg import recvMsg

def processRois(Image):
	#For serey. The rois shall be returned here, this way:
	#return 12, {} #number of regions and array of regions

	#initialising values we have to append
	regions_number = 0;
	regions = [];
	currentRegion = RegionOfInterest()

	#process image


	#return results
	return regions_number, regions


#Callback function - called everytime a new message is recieved
def callback(data):

	recv_msg = data;

	#Parse data we just recieved
	timeNow = rospy.get_rostime()
	recvTime = recv_msg.image.header.stamp
	timeRequired = (timeNow.nsecs - recvTime.nsecs)/1e6 #convert time to miliseconds

	seq  = recv_msg.image.header.seq
	fid  = recv_msg.image.header.frame_id
	imux = recv_msg.imu.imu.angular_velocity.x
	imuy = recv_msg.imu.imu.angular_velocity.y
	imuz = recv_msg.imu.imu.angular_velocity.z

	#Print received data
	print '[%s ms][%s][%s][IMU X: %s Y: %s Z: %s]' % (timeRequired, seq, fid, imux, imuy, imuz)

	#start building message we have to publish
	send_msg = sendMsg()

	#frame is the same; we just forward it
	send_msg.image = recv_msg.image

	#updating header stamp
	send_msg.image.header.stamp = rospy.get_rostime()

	#IMU data also remain the same; we just forward to subscribers
	send_msg.imu = recv_msg.imu

	#appending ROIs to our message
	#We have not to append number of ROIs to: send_msg.rois.nr and array at send_msg.rois.rois
	send_msg.rois.nr, send_msg.rois.rois = processRois(send_msg.image)

	#sending message
	hPublisher.publish(send_msg)

#Main function
if __name__ == '__main__':

	#Initializing node
	rospy.init_node('roi_node', anonymous=True)

	#Defining publisher and subscriber handles
	hSubscriber = rospy.Subscriber("camera_and_imu", recvMsg, callback)			    #Subscriber handler
	hPublisher  = rospy.Publisher("camera_and_imu_and_roi", sendMsg, queue_size=1)	#Publisher handler

	rospy.spin()
