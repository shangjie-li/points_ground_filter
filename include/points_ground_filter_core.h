#pragma once

#include <ros/ros.h>
#include <limits.h>
#include <float.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>

#include "opencv2/opencv.hpp"

// To disable PCL compile lib and use PointXYZICustom
#define PCL_NO_PRECOMPILE

namespace pgf
{
/** Euclidean coordinate, including intensity and index in custom array. */
struct PointXYZICustom
{
    PCL_ADD_POINT4D; // quad-word XYZ
    float intensity; // laser intensity reading
    
    int16_t x_idx;
    int16_t y_idx;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;
};

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(pgf::PointXYZICustom, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(int16_t, x_idx, x_idx)(int16_t, y_idx, y_idx))

#define PI 3.1415926

class PointsGroundFilter
{
private:
    std::string sub_topic_;
    std::string pub_ground_topic_;
    std::string pub_no_ground_topic_;
    
    bool show_points_size_;
    bool show_time_;

    float max_x_;
    float max_y_;
    float x_divider_;
    float y_divider_;
    float local_slope_threshold_;
    
    bool ground_filter_mode_;
    float ground_meank_;
    float ground_stdmul_;
    
    bool no_ground_filter_mode_;
    float no_ground_meank_;
    float no_ground_stdmul_;

    ros::Subscriber sub_;
    ros::Publisher pub_ground_, pub_no_ground_;
    
    cv::Mat height_mat_;
    cv::Mat num_mat_;
    cv::Mat slope_mat_;
    
    void convertPointCloud(pcl::PointCloud<pgf::PointXYZICustom>::Ptr pc);
    
    void classifyPointCloud(const pcl::PointCloud<pgf::PointXYZICustom>::Ptr pc,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_no_ground);
                        
    void callback(const sensor_msgs::PointCloud2ConstPtr pc_msg);
public:
    PointsGroundFilter(ros::NodeHandle& nh);
    ~PointsGroundFilter();
};

