#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <depth_pkg/pos.h>
#include <depth_pkg/multi_pos.h>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include<darknet_ros_msgs/ObjectCount.h>
#include <geometry_msgs/Twist.h>
#include<vector>
#include "ros/time.h"
#include "depth_pkg/KalmanFilter1.h"

#include<stdio.h>
#include<math.h>
#include<algorithm>
#include <deque>

using namespace cv;
using namespace std;
using namespace Eigen;

KalmanFilter1 kf;

cv::Mat depth_pic;
vector<double> d_value(100);


vector<double> x_cent(100);
vector<double> y_cent(100);
VectorXd x_out(4, 1);

ros::Publisher obstacle_pos_pub;
ros::Publisher target_pos_pub;
ros::Publisher vel_pub;
int len_of_arr;
float fx = 343.4963684082031;
float fy = 343.4963684082031;
float cx = 320;
float cy = 180;
double last_tsp = 0;
bool TrackFlag = false;


pair<float,float> real_world_cal(float Z, float u, float v)
{
    pair<float,float> coordinate;
    Matrix3f intrinsics;
    intrinsics << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    MatrixXf pixel(3,1);
    pixel(0,0) = u;
    pixel(1,0) = v;
    pixel(2,0) = 1;
    MatrixXf real_world;
    real_world = intrinsics.inverse()*Z*pixel;
    coordinate = make_pair(real_world(0,0),real_world(1,0));

    return coordinate;
}

int FilterSet(double now_tsp, double Sx, double Sy, double Sz)
{
    
    if (!kf.IsInitialized())
    {
        last_tsp = now_tsp;
        VectorXd x_in(6,1);
        x_in << Sx, Sy, Sz, 0, 0, 0;
        kf.Initialization(x_in);
        MatrixXd P_in(6,6);
        P_in << 1,0,0,0,0,0,
                0,1,0,0,0,0,
                0,0,1,0,0,0,
                0,0,0,100,0,0,
                0,0,0,0,100,0,
                0,0,0,0,0,100;
        kf.SetP(P_in);
        MatrixXd Q_in(6,6);
        Q_in << 1,0,0,0,0,0,
                0,1,0,0,0,0,
                0,0,1,0,0,0,
                0,0,0,1,0,0,
                0,0,0,0,1,0,
                0,0,0,0,0,1;
        kf.SetQ(Q_in);
        MatrixXd H_in(3,6);
        H_in << 1,0,0,0,0,0,
                0,1,0,0,0,0,
                0,0,1,0,0,0;
        kf.SetH(H_in);
        MatrixXd R_in(3,3);
        R_in << 0.1,0,0,
                0,0.1,0,
                0,0,0.1;
        kf.SetR(R_in);
        return 0;
    }

    double dt = now_tsp-last_tsp;
    MatrixXd F_in(6,6);
    F_in << 1,0,0,dt,0,0,
            0,1,0,0,dt,0,
            0,0,1,0,0,dt,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1;
    kf.SetF(F_in);
    kf.Prediction();
    VectorXd z(3,1);
    z << Sx, Sy, Sz;
    kf.MeasurementUpdate(z);
    x_out = kf.GetX();
    last_tsp = now_tsp;
}

void object_callback(darknet_ros_msgs::ObjectCount msg)
{
    len_of_arr = msg.count;
}

void cent_x_callback(std_msgs::Float64MultiArray msg_x)
{
    x_cent.resize(len_of_arr);
    x_cent = msg_x.data;
}

void cent_y_callback(std_msgs::Float64MultiArray msg_y)
{
    y_cent.resize(len_of_arr);
    y_cent = msg_y.data;
}

void depth_callback(const sensor_msgs::Image::ConstPtr &msg)
{
    // if (TrackFlag) cout<<"target was found with "<<len_of_arr-1<<" obstacle, start tracking!"<<endl;
    // else cout<<"target not found while "<<len_of_arr<<" obstacle was found"<<endl;
    cv_bridge::CvImagePtr Dest;
    Dest = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
    depth_pic = Dest->image;
    int j,i;
    for (j=0;j<len_of_arr;j++)
    {
        short d = depth_pic.at<short>(y_cent[j],x_cent[j]);
        d_value[j] = float(d)/1000;
        // ROS_INFO("NO.%d node's depth is %.2fmm!",j,d_value[j]);
    }
    depth_pkg::multi_pos multi_pos_msg;
    for (i=0;i<len_of_arr;i++)
    {
        pair<float,float> coordinate = real_world_cal(d_value[i],x_cent[i],y_cent[i]);
 //       ROS_INFO("NO.%d node's position is X=%.2f,Y=%.2f,Z=%.2f!",i+1,coordinate.first,coordinate.second,d_value[i]);
        depth_pkg::pos XYZ_msg;
        XYZ_msg.real_X = coordinate.first;
        XYZ_msg.real_Y = coordinate.second; 
        XYZ_msg.real_Z = d_value[i];
        XYZ_msg.label = i;
        multi_pos_msg.MultiPos.push_back(XYZ_msg);
    }
    obstacle_pos_pub.publish(multi_pos_msg);
}

void td_callback (std_msgs::Float64MultiArray msg1)
{
    float u,v,X,Y;
    float vx,vy,vz;
    float Z = 0;
    vector<double> msg_track(3);
    msg_track = msg1.data;
    ros::Time time1 = ros::Time::now();
    double time2 = time1.toSec();
    Z = msg_track[0];
    u = msg_track[1];
    v = msg_track[2];

    if(Z!=0) 
    {
        TrackFlag = true;
        pair<float,float> coordinate = real_world_cal(Z,u,v);

        X = coordinate.first;
        Y = coordinate.second;
        
    }

    // if(TimeStamp.front()!=0)
    // {
        FilterSet(time2, X, Y, Z);

        X = x_out(0);
        Y = x_out(1);
        Z = x_out(2);
        vx = x_out(3);
        vy = x_out(4);
        vz = x_out(5);  


//        ROS_INFO("target node's position is X=%.2f,Y=%.2f,Z=%.2f!",X,Y,Z);
        depth_pkg::pos tg_msg;
        tg_msg.real_X = X;
        tg_msg.real_Y = Y;
        tg_msg.real_Z = Z;
        target_pos_pub.publish(tg_msg);    

        cout <<"vx= "<<vx<<"    vy= "<<vy<<"     vz= "<<vz<<endl;
        geometry_msgs::Twist line_speed;
        line_speed.linear.x = vx;
        line_speed.linear.y = vy;
        line_speed.linear.z = vz;
        vel_pub.publish(line_speed);
    }

//}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "depth_node");

    ros::NodeHandle n;
    ros::Subscriber cent_x_sub = n.subscribe("/cent_loc_msgs_x", 10, cent_x_callback);
    ros::Subscriber cent_y_sub = n.subscribe("/cent_loc_msgs_y", 10, cent_y_callback);
    ros::Subscriber sub_1 = n.subscribe("/darknet_ros/found_object",10,object_callback);
    // ros::Subscriber depth_sub = n.subscribe<sensor_msgs::Image>("/camera/depth_aligned_to_color_and_infra1/image_raw",10,depth_callback);
    ros::Subscriber depth_sub = n.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw",10,depth_callback);
    ros::Subscriber track_sub = n.subscribe("/tracker/point_info",10,td_callback);
    obstacle_pos_pub = n.advertise<depth_pkg::multi_pos>("position/obstacle_pos",10);
    target_pos_pub = n.advertise<depth_pkg::pos>("position/target_pose",10);
    vel_pub = n.advertise<geometry_msgs::Twist>("vel/target_vel",10);
    


    while (ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}
