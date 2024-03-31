#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/aruco.hpp>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <chrono>

cv::Mat frame;  //初始化frame时不指定分辨率和类型，这样保证程序可以接受任意分辨率的图片，以及rgb图或者灰度图。  
//cv::Mat frame = cv::Mat( 480, 640, CV_8UC3, cv::Scalar(0));;

void cam_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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
    ros::init(argc, argv, "aruco_det_node");
    ros::NodeHandle n;

    // 创建一个发布图像消息的发布者
    ros::Publisher aruco_det_image_pub = n.advertise<sensor_msgs::Image>("aruco_det_image", 10);
    // 订阅灰度图像话题
    ros::Subscriber cam_sub = n.subscribe("/usb_cam/image_raw", 1, cam_imageCallback);

    ros::Publisher aruco_det_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/aruco/pose", 10);

    // 创建一个CvBridge对象
    cv_bridge::CvImage cv_image;

    int aruco_id = 19;  //指定要检测的aruco二维码id  
    float aruco_length = 0.05;  //二维码边长，单位米

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

        //没有frame.empty()这个判断，frame为空时cvtColor会报错。  
        if(!frame.empty())
        {
        // 转换为灰度图像
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        //opencv已经有现成的aruco二维码检测函数了，分为了两个步骤，先提前角点，再根据二维码角点估计相对位姿，分成两个函数我仔细想想也是有必要的，对于那种一张图里面有多张二维码时自己好灵活处理一些，同时，也方便了我们自己想弄PnP的，正好就直接用提取好的角点再用solvePnP函数算即可。

        // 检测ArUco二维码
        std::vector<int> markerIds;  //因为可能检测到多个aruco二维码，所以是vector类型。  
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        // 绘制检测结果
        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            //float aruco_length = 0.05;  //二维码边长，单位米
            // 估计相机姿态
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, aruco_length, cameraMatrix, distCoeffs, rvecs, tvecs);

            //tvecs是aruco二维码中心点在相机坐标系下的坐标 参考：https://blog.csdn.net/weixin_46450859/article/details/133799263  
            //rvecs应该是二维码坐标系在相机系下的旋转
            // 绘制相对位姿
            for (int i = 0; i < markerIds.size(); ++i)
            {
                cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            }

            // 要查找的二维码id
            //int aruco_id = 19;

            // 使用 std::find 在 vector 中查找值aruco_id
            auto it = std::find(markerIds.begin(), markerIds.end(), aruco_id);

            // 检查是否找到
            if (it != markerIds.end()) {
               // 找到值的索引
               int index = std::distance(markerIds.begin(), it);
               std::cout << "aruco_id " << aruco_id << " found at index: " << index << std::endl;

            geometry_msgs::PoseStamped aruco_det_pose;

            aruco_det_pose.pose.position.x = tvecs[index][0];
            aruco_det_pose.pose.position.y = tvecs[index][1];
            aruco_det_pose.pose.position.z = tvecs[index][2];

            aruco_det_pose.header.stamp = ros::Time::now();

            aruco_det_pose_pub.publish(aruco_det_pose);  

            } else {
               std::cout << "aruco_id " << aruco_id << " not found in the vector markerIds." << std::endl;
            }

        }

        cv_image.image = frame;
        //cv_image.encoding = "mono8";  //灰度图这里注意用mono8
        //cv_image.encoding = "mono16";
        cv_image.encoding = "bgr8";
        sensor_msgs::ImagePtr img_msg = cv_image.toImageMsg();

        aruco_det_image_pub.publish(img_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}





