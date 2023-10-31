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
#include <queue>

class Events2Image 
{
    private:
	ros::NodeHandle nh;
        ros::Publisher  image_pub;
        ros::Subscriber event_sub;   
        std::queue <dvs_msgs::EventArray> ev_queue;
        int dequeue_pkts = 300;

    public:
        void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
        {
          //Store Events
          ev_queue.push(*msg);

          //If Packets Store  enough
          if (ev_queue.size() > dequeue_pkts)
          {
            cv_bridge::CvImage cv_image;

            //Make image
            cv_image.encoding = "bgr8";
            cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
            cv_image.image = cv::Scalar(0,0,0);
            //Open Each Packets
            for (int pkt_num = 0; pkt_num < dequeue_pkts; pkt_num++)
            {
              dvs_msgs::EventArray e_pkt = ev_queue.front(); 
              ev_queue.pop();
              //Use Avenrage Time-Stamp
              if (pkt_num == dequeue_pkts / 2)
              {
                cv_image.header.stamp =e_pkt.events[e_pkt.events.size()/2].ts; 
              }
              
              for (int i = 0; i < e_pkt.events.size(); ++i)
              {
                const int x = e_pkt.events[i].x;
                const int y = e_pkt.events[i].y;
        
                cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
                    e_pkt.events[i].polarity == true ? cv::Vec3b(0, 255, 0) : cv::Vec3b(255, 255, 255));
                //RED-BLUE
                //cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
                //   e_pkt.events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
              }
            }
              image_pub.publish(cv_image.toImageMsg());
            }
        
        }
        
        Events2Image()
        {
          image_pub = nh.advertise<sensor_msgs::Image>("eventsImage", 1);
          event_sub = nh.subscribe("/dvs/events", 1, &Events2Image::eventsCallback, this);
        }

};



int main(int argc, char** argv)
{
	ros::init(argc,argv, "event2image");
        Events2Image subpub;
	ros::spin();
	return 0;
}
