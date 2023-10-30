// This file is part of DVS-ROS - the RPG DVS ROS Package

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv, "event2image");
	ros::NodeHandle nh;
        // setup subscribers and publishers
        ros::Subscriber event_sub = nh.subscribe("events", 1, &eventsCallback);
        ros::Publisher  image_pub = nh.advertise("eventsImage", 1);

	ros::spin();
	return 0;
}


void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  cv_bridge::CvImage cv_image;
  if (msg->events.size() > 0)
  {
    cv_image.header.stamp = msg->events[msg->events.size()/2].ts;
  }
  //Make image
  cv_image.encoding = "bgr8";
  cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
  cv_image.image = cv::Scalar(0,0,0);

  for (int i = 0; i < msg->events.size(); ++i)
  {
    const int x = msg->events[i].x;
    const int y = msg->events[i].y;

    cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
        msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
  }
  image_pub.publish(cv_image.toImageMsg());
}


