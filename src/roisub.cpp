#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "roi/sendMsg.h"

#include <iostream>

void image_callback(const roi::sendMsg &recvMsg)
{
    try
    {
        //Display how much time was neccessary for this message to reach this node
        long t_now = ros::Time::now().toNSec();
        long t_recv = recvMsg.image.header.stamp.toNSec();
        std::cout << "[" << (double)(t_now - t_recv)/1e6 << " ms] ";

        //Display transmission sequence data
        std::cout <<"[SEQ: " << recvMsg.image.header.seq << "]";

        //Display frame ID data
        std::cout << "[F_ID: " << recvMsg.image.header.frame_id << "]";

        //Display some IMU data
        std::cout << "[IMU X:" << recvMsg.imu.imu.angular_velocity.x << " ";
        std::cout << "Y: " << recvMsg.imu.imu.angular_velocity.y << " ";
        std::cout << "Z: " << recvMsg.imu.imu.angular_velocity.z << "]";

        //Display ROIs
        std::cout << "[Rois_nr: " << recvMsg.rois.nr << "]" << std::endl;

        cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(recvMsg.image, "bgr8");

        cv::imshow("view", cvImg->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout <<"Error: [" << recvMsg.image.header.seq << "]" << std::endl;
       //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
		//Initializare ros
		ros::init(argc, argv, "subscriberToMessage");
		ros::NodeHandle n;

		//Starting the window to view the frames
		cv::namedWindow("view");
		cv::startWindowThread();

		//Abonare la topicul specificat
		ros::Subscriber subscriber = n.subscribe("camera_and_imu_and_roi", 1, image_callback);
		ros::spin();

		cv::destroyWindow("view");

		return 0;
}
