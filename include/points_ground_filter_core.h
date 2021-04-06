#pragma once

#include <ros/ros.h>
#include <limits.h>
#include <float.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#define PI 3.1415926

class PointsGroundFilter
{
private:
    std::string sub_topic_;
    std::string pub_ground_topic_;
    std::string pub_no_ground_topic_;
    std::string pub_marker_topic_;
    
    bool show_points_size_;
    bool show_time_;

    float sensor_height_;
    float radius_divider_;
    float theta_divider_;
    float local_slope_threshold_;
    float general_slope_threshold_;
    float curb_height_threshold_;
    
    bool ground_filter_mode_;
    float ground_meank_;
    float ground_stdmul_;
    
    bool no_ground_filter_mode_;
    float no_ground_meank_;
    float no_ground_stdmul_;

    ros::Subscriber sub_;
    ros::Publisher pub_ground_, pub_no_ground_, pub_marker_;
    
    struct PointXYZRTColor
    {
        pcl::PointXYZ point;

        float radius; //XY平面极坐标系的半径
        float theta;  //XY平面极坐标系的极角

        size_t radius_idx; //径向索引
        size_t theta_idx;  //周向索引

        size_t original_idx; //在原始点云中的索引
    };

    struct PointR
    {
        geometry_msgs::Point point;

        float radius; //XY平面极坐标系的半径
    };

    typedef std::vector<PointXYZRTColor> PointCloudXYZRTColor;
    
    void callback(const sensor_msgs::PointCloud2ConstPtr &in);
    
    void convert_XYZ_to_XYZRTColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
                        std::vector<PointCloudXYZRTColor> &out_pc);
    
    void classify_pc(std::vector<PointCloudXYZRTColor> &in_pc,
                        pcl::PointIndices &ground_indices,
                        pcl::PointIndices &no_ground_indices);

    void publish_pc(const ros::Publisher &pub,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr,
                        const std_msgs::Header &header);
    
    void publish_marker(const ros::Publisher &pub,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
                        visualization_msgs::Marker &region,
                        std_msgs::Header header);

public:
    PointsGroundFilter(ros::NodeHandle &nh);
    ~PointsGroundFilter();
    void Spin();

};



