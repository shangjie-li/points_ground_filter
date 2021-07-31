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

// To disable PCL compile lib and use PointXYZIR
#define PCL_NO_PRECOMPILE

namespace pgf
{
/** Euclidean coordinate, including intensity and ring number. */
struct PointXYZIR
{
    PCL_ADD_POINT4D;                // quad-word XYZ
    float intensity;                // laser intensity reading
    uint16_t ring;                  // laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;
};

// Register custom point struct according to PCL
// POINT_CLOUD_REGISTER_POINT_STRUCT(pgf::PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))
POINT_CLOUD_REGISTER_POINT_STRUCT(pgf::PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))

class PointsGroundFilter
{
private:
    ros::Subscriber sub_point_cloud_;
    ros::Publisher pub_ground_;
    ros::Publisher pub_no_ground_;
  
    std::string sub_topic_;
    std::string pub_ground_topic_;
    std::string pub_no_ground_topic_;
    
    bool show_points_size_;
    bool show_time_;
    
    bool crop_range_mode_;
    float range_front_;
    float range_rear_;
    float range_left_;
    float range_right_;
  
    float sensor_height_;
    float seeds_distance_threshold_;
    float ground_distance_threshold_;
    int num_lpr_;
    
    int seg_num_front_;
    int seg_num_rear_;
    std::vector<float> seg_distance_front_;
    std::vector<float> seg_distance_rear_;
    
    void estimatePlane(const pcl::PointCloud<pgf::PointXYZIR>::Ptr pc,
                       Eigen::MatrixXf& normal,
                       float& d);
    void extractInitialSeeds(const pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_sorted,
                             pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_seeds);
    void classifyPointCloudInSegment(pcl::PointCloud<pgf::PointXYZIR>::Ptr pc,
                                     pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_ground,
                                     pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_no_ground);
    void classifyPointCloud(pcl::PointCloud<pgf::PointXYZIR>::Ptr pc,
                            pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_ground,
                            pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_no_ground);
    void cropPointCloud(const pcl::PointCloud<pgf::PointXYZIR>::Ptr pc,
                        pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_cropped);
    void callback(const sensor_msgs::PointCloud2ConstPtr pc_msg);

public:
    PointsGroundFilter(ros::NodeHandle& nh);
    ~PointsGroundFilter();
};


