//
// Created by root on 11/2/24.
//

#ifndef BUILD_IMAGEPROJECTION_H
#define BUILD_IMAGEPROJECTION_H

#include "utils/ParamServer.h"
#include "utils/imuUtils.h"
#include "utils/pclUtils.h"
#include "utils/utils.h"

#include "utils/pclType.h"
#include "lio_sam/msg/cloud_info.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <opencv2/opencv.hpp>

const int queueLength = 2000;

class ImageProjection : public ParamServer {
private:
    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<lio_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    sensor_msgs::msg::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int ringFlag = 0;
    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

    vector<int> columnIdnCountVec;

public:
    ImageProjection(const rclcpp::NodeOptions & options);
    void allocateMemory();
    void resetParameters();
    ~ImageProjection();
    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg);
    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg);
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& laserCloudMsg);
    bool deskewInfo();
    void imuDeskewInfo();
    void odomDeskewInfo();
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur);
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur);
    PointType deskewPoint(PointType *point, double relTime);
    void projectPointCloud();
    void cloudExtraction();
    void publishClouds();
};

#endif //BUILD_IMAGEPROJECTION_H
