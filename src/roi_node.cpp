#include "ros/ros.h"

//OpenCv and the company for working and displaying frames
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//We need messages definitions in order to be able to recieve/dend
#include "roi/recvMsg.h"                    //Received message format
#include "roi/sendMsg.h"                    //Sending message format
#include "sensor_msgs/RegionOfInterest.h"   //Region of interest format for ROIs array
#include "sensor_msgs/Image.h"              //Required!!

#include <iostream> //std::cout
#include <vector>   //regions vector

//Global publisher handler
ros::Publisher publisher;

//Functions used to detect region of interests
//Input  : *img
//Output : *number, *rois
void processRegionsOfInterest(const sensor_msgs::Image *img, uint32_t *number, std::vector<sensor_msgs::RegionOfInterest> *rois)
{
    //Initialize params we have to return
    *number = 0;

    //Handle image in OpenCV format
    cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(*img, "bgr8");

    //Process ROIS here...

}

//Callback function for subscriber. This is called everytime a message is received from subscriber
void image_callback(const roi::recvMsg &recvMsg)
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
        std::cout << "Z: " << recvMsg.imu.imu.angular_velocity.z << "]" << std::endl;

        //Starging to build the message which have to be send
        //The new message should have appended regions of interest details
        roi::sendMsg sendMsg;    //message we are about to publish format;

        //The image frame of the message we will send is the same. We just forward to publisher
        sendMsg.image = recvMsg.image;

        //Reset timestamp
        sendMsg.image.header.stamp = ros::Time::now();

        //Appending IMU data si the same also. We just forward to publisher
        sendMsg.imu = recvMsg.imu;

        //Call function to process ROIs - INPUT: cvImg; OUTPUT: regions_number and regions
        processRegionsOfInterest(&recvMsg.image, &sendMsg.rois.nr, &sendMsg.rois.rois);

        //New message is builded and ROIs are appended. Now is have to be published
        publisher.publish(sendMsg);

        //Display image in a new window - debugging only
        //        cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(recvMsg.image, "bgr8");
        //        cv::imshow("view", cvImg->image);
        //        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        //If recieved frame is not valid, throw error in console.
        std::cout <<"Error: [" << recvMsg.image.header.seq << "]" << std::endl;
    }
}

int main(int argc, char **argv)
{
    //Initializare ros
    ros::init(argc, argv, "roiDetector");

    //Creating nodes
    ros::NodeHandle subscribeNode;
    ros::NodeHandle publishNode;

    //Starting the window to view the frames - only for debugging
    //cv::namedWindow("view");
    //cv::startWindowThread();

    //Creating subscription and publishing handlers
    publisher = publishNode.advertise<roi::sendMsg>("camera_and_imu_and_roi", 1) ;               //publisher
    ros::Subscriber subscriber = subscribeNode.subscribe("camera_and_imu", 1, image_callback);   //subscriber

    //Make sure we recieve until booth nodes are shutdown
    ros::spin();

    cv::destroyWindow("view");
    return 0;
}
