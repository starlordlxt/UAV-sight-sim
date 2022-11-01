#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
   //自定义一个消息类型
 
using namespace std;

using namespace cv;
 
cv::Mat depth_pic;        //定义全局变量，图像矩阵Ｍat形式
float d_value;

void depthCallback(const sensor_msgs::Image::ConstPtr&msg)
{
    cv_bridge::CvImagePtr Dest ;
    Dest = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);

    depth_pic = Dest->image;
    cout<<"Output some info about the depth image in cv format"<<endl;
    cout<< "Rows of the depth iamge = "<<depth_pic.rows<<endl;                       //获取深度图的行数height
    cout<< "Cols of the depth iamge = "<<depth_pic.cols<<endl;                           //获取深度图的列数width
    cout<< "Type of depth_pic's element = "<<depth_pic.type()<<endl;             //深度图的类型
    ushort d = depth_pic.at<ushort>(depth_pic.rows/2,depth_pic.cols/2);           //读取深度值，数据类型为ushort单位为ｍｍ
    d_value = float(d)/1000 ;      //强制转换
    cout<< "Value of depth_pic's pixel= "<<d_value<<endl;    //读取深度值

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "stream_pub");               // ros节点初始化
    ros::NodeHandle nh;                                           //创建节点句柄
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/camera/depth_aligned_to_color_and_infra1/image_raw",1,depthCallback);   //订阅深度图像
    //ros::Subscriber element_sub = nh.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw",100,pixelCallback);     //订阅像素点坐标

   //初始化深度值
    ros::Rate rate(20.0);    //设定自循环的频率
    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    ros::Duration(10).sleep();    //设定自循环的频率
    return 0 ;
}

