#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/aruco.hpp>

#include <iostream>
#include <chrono>


cv::Mat frame;

void cam_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        //cv::Mat left_gray_image = cv_ptr->image;
        frame = cv_ptr->image;

        // 在这里可以对灰度图像进行处理
        // 比如显示图像信息
        ROS_INFO("Received a %d x %d image", frame.cols, frame.rows);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher_node");
    ros::NodeHandle n;

    // 创建一个发布图像消息的发布者
    ros::Publisher aruco_det_image_pub = n.advertise<sensor_msgs::Image>("image_topic", 10);
    // 订阅灰度图像话题
    ros::Subscriber cam_sub = n.subscribe("/camera/infra1/image_rect_raw", 1, cam_imageCallback);

    // 创建一个CvBridge对象
    cv_bridge::CvImage cv_image;

    // 定义ArUco字典和参数
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();


    double fx = 406.932130;
    double fy = 402.678201;
    double cx = 316.629381;
    double cy = 242.533947;

    double k1 = 0.039106;
    double k2 = -0.056494;
    double p1 = -0.000824;
    double p2 = 0.092161;
    double k3 = 0.0;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
    fx, 0, cx,
    0, fy, cy,
    0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

    ros::Rate loop_rate(10);  // 设置发布频率为1Hz
    while (ros::ok())
    {

        // 转换为灰度图像
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 检测ArUco二维码
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        // 绘制检测结果
        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            // 估计相机姿态
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

            // 绘制相对位姿
            for (int i = 0; i < markerIds.size(); ++i)
            {
                cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            }
        }

    cv_image.image = frame;
    //cv_image.encoding = "mono8";  //灰度图这里注意用mono8
    //cv_image.encoding = "mono16";
    cv_image.encoding = "bgr8";
    sensor_msgs::ImagePtr img_msg = cv_image.toImageMsg();

        aruco_det_image_pub.publish(img_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}





