#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>
#include<vector>
#include <std_msgs/MultiArrayDimension.h>
#include<darknet_ros_msgs/BoundingBox.h>
#include<darknet_ros_msgs/BoundingBoxes.h>
#include<darknet_ros_msgs/ObjectCount.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<algorithm>
using namespace cv;
using namespace std;
ros::Publisher cent_loc_pub_x;
ros::Publisher cent_loc_pub_y;


int boxes_num;

void object_callback(darknet_ros_msgs::ObjectCount msg)
{
    boxes_num = msg.count;
}

void boxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    int i,j;
    std_msgs::Float64MultiArray msg_x_cent;
    std_msgs::Float64MultiArray msg_y_cent;
    vector<double> x_cent(boxes_num),y_cent(boxes_num);
    double xmin, ymin, xmax, ymax;
    for(i=0;i<boxes_num;i++)
    {
        xmin = msg->bounding_boxes[i].xmin;
	    ymin = msg->bounding_boxes[i].ymin;
	    xmax = msg->bounding_boxes[i].xmax;
	    ymax = msg->bounding_boxes[i].ymax;
        x_cent[i] = (xmin+xmax)/2;
        y_cent[i] = (ymin+ymax)/2;
         ROS_INFO("no.%d's center is (%.2f,%.2f)",i,x_cent[i],y_cent[i]);
    }

        msg_x_cent.data = x_cent;
        msg_y_cent.data = y_cent;
    
    cent_loc_pub_x.publish(msg_x_cent);
    cent_loc_pub_y.publish(msg_y_cent);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "center_loc_node");
    ros::NodeHandle n;
    ros::Time time = ros::Time::now();
    ros::Subscriber sub_1 = n.subscribe("/darknet_ros/found_object",10,object_callback);
    ros::Subscriber sub_2 = n.subscribe("/darknet_ros/bounding_boxes",10,boxes_callback);
    cent_loc_pub_x = n.advertise<std_msgs::Float64MultiArray>("/cent_loc_msgs_x",1000);
    cent_loc_pub_y = n.advertise<std_msgs::Float64MultiArray>("/cent_loc_msgs_y",1000);

while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
