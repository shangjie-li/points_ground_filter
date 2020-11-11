#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#define PI 3.1415926

class PointsGroundFilter
{

private:
    std::string sub_topic_;
    std::string pub_ground_topic_;
    std::string pub_no_ground_topic_;
    
    bool show_points_size_;

    float radial_divider_angle_;
    float sensor_height_;
    
    float concentric_divider_distance_;
    float local_max_slope_; //max slope of the ground between points, degree
    float general_max_slope_; //max slope of the ground in entire point cloud, degree
    
    float min_height_threshold;
    float reclass_distance_threshold;
    size_t radial_dividers_num;

    ros::Subscriber sub_pointcloud;
    ros::Publisher pub_pointcloud_ground, pub_pointcloud_no_ground;
    
    struct PointXYZRTColor
    {
        pcl::PointXYZ point;

        float radius; //cylindric coords on XY Plane
        float theta;  //angle deg on XY plane

        size_t radial_div;     //index of the radial divsion to which this point belongs to
        size_t concentric_div; //index of the concentric division to which this points belongs to

        size_t original_index; //index of this point in the source pointcloud
    };
    typedef std::vector<PointXYZRTColor> PointCloudXYZRTColor;
    
    void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
    
    void XYZ_to_RTZColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
                        PointCloudXYZRTColor &out_organized_points,
                        std::vector<pcl::PointIndices> &out_radial_divided_indices,
                        std::vector<PointCloudXYZRTColor> &out_radial_ordered_clouds);
    
    void classify_pc(std::vector<PointCloudXYZRTColor> &in_radial_ordered_clouds,
                   pcl::PointIndices &out_ground_indices,
                   pcl::PointIndices &out_no_ground_indices);
    
    void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);

public:
    PointsGroundFilter(ros::NodeHandle &nh);
    ~PointsGroundFilter();
    void Spin();

};



