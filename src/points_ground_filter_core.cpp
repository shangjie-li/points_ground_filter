#include "points_ground_filter_core.h"

PointsGroundFilter::PointsGroundFilter(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/rslidar_points");
    nh.param<std::string>("pub_ground_topic", pub_ground_topic_, "/rslidar_points_ground");
    nh.param<std::string>("pub_no_ground_topic", pub_no_ground_topic_, "/rslidar_points_no_ground");
    
    nh.param<bool>("show_points_size", show_points_size_, false);

    nh.param<float>("sensor_height", sensor_height_, 1.8);
    nh.param<float>("radial_divider_angle", radial_divider_angle_, 0.2);
    nh.param<float>("concentric_divider_distance", concentric_divider_distance_, 0.1);
    

    nh.param<float>("local_threshold_ratio", local_threshold_ratio_, 0.01);
    nh.param<float>("min_local_threshold", min_local_threshold_, 0.05);
    nh.param<float>("general_threshold_ratio", general_threshold_ratio_, 0.02);
    nh.param<float>("min_general_threshold", min_general_threshold_, 0.15);
    nh.param<float>("reclass_distance", reclass_distance_, 0.2);
    
    nh.param<bool>("ground_filter_mode", ground_filter_mode_, false);
    nh.param<float>("ground_meank", ground_meank_, 10);
    nh.param<float>("ground_stdmul", ground_stdmul_, 0.2);
    
    nh.param<bool>("no_ground_filter_mode", no_ground_filter_mode_, false);
    nh.param<float>("no_ground_meank", no_ground_meank_, 10);
    nh.param<float>("no_ground_stdmul", no_ground_stdmul_, 0.2);
    
    sub_pointcloud = nh.subscribe(sub_topic_, 1, &PointsGroundFilter::point_cb, this);
    pub_pointcloud_ground = nh.advertise<sensor_msgs::PointCloud2>(pub_ground_topic_, 1);
    pub_pointcloud_no_ground = nh.advertise<sensor_msgs::PointCloud2>(pub_no_ground_topic_, 1);
    
    radial_dividers_num = ceil(360 / radial_divider_angle_);
    
    ros::spin();
}

PointsGroundFilter::~PointsGroundFilter(){}

void PointsGroundFilter::Spin(){}

void PointsGroundFilter::XYZ_to_RTZColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
                                   PointCloudXYZRTColor &out_organized_points,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZRTColor> &out_radial_ordered_clouds)
{
    out_organized_points.resize(in_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num);
    out_radial_ordered_clouds.resize(radial_dividers_num);

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        PointXYZRTColor new_point;
        auto radius = (float)sqrt(
            in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / PI;
        if (theta < 0)
        {
            theta += 360;
        }
        //角度的微分
        auto radial_div = (size_t)floor(theta / radial_divider_angle_);
        //半径的微分
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        out_organized_points[i] = new_point;

        //按角度的微分组织射线
        out_radial_divided_indices[radial_div].indices.push_back(i);

        out_radial_ordered_clouds[radial_div].push_back(new_point);

    }

#pragma omp for
    //将同一根射线上的点按照半径（距离）排序
    for (size_t i = 0; i < radial_dividers_num; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZRTColor &a, const PointXYZRTColor &b) { return a.radius < b.radius; });
    }
}

void PointsGroundFilter::classify_pc(std::vector<PointCloudXYZRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();
#pragma omp for
    //遍历每一根射线
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++)
    {
        float prev_radius = 0.f;
        float prev_height = -sensor_height_;
        bool prev_ground = false;
        bool current_ground = false;
        //遍历射线上的每一个点
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) 
        {
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            //当前点与前一个点的距离
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            //当前点与前一个点高度差的阈值
            float locoal_height_threshold = local_threshold_ratio_ * in_radial_ordered_clouds[i][j].radius + min_local_threshold_;
            //当前点与地面距离的阈值
            float general_height_threshold = general_threshold_ratio_ * in_radial_ordered_clouds[i][j].radius + min_general_threshold_;

            //若当前点与前一个点高度相近
            if (current_height <= (prev_height + locoal_height_threshold) && current_height >= (prev_height - locoal_height_threshold))
            {
                if (!prev_ground)
                {
                    //判断当前点与地面距离
                    if (current_height <= (-sensor_height_ + general_height_threshold) && current_height >= (-sensor_height_ - general_height_threshold))
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            //若当前点与前一个点高度不相近
            else
            {
                //判断是否存在高度突变
                //判断当前点与地面距离
                if (points_distance > reclass_distance_ &&
                    (current_height <= (-sensor_height_ + general_height_threshold) && current_height >= (-sensor_height_ - general_height_threshold)))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }

            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}

void PointsGroundFilter::publish_cloud(const ros::Publisher &in_publisher,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                                const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void PointsGroundFilter::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    
    if (show_points_size_)
    {
        std::cout<<"points_size of current_pc_ptr:"<<current_pc_ptr->points.size()<<std::endl;
    }

    PointCloudXYZRTColor organized_points;
    std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<PointCloudXYZRTColor> radial_ordered_clouds;

    XYZ_to_RTZColor(current_pc_ptr, organized_points,
                     radial_division_indices, radial_ordered_clouds);

    pcl::PointIndices ground_indices, no_ground_indices;

    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractIndices<pcl::PointXYZ> extract_ground;
    extract_ground.setInputCloud(current_pc_ptr);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
    extract_ground.filter(*ground_cloud_ptr);

    extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
    extract_ground.filter(*no_ground_cloud_ptr);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    //for ground_cloud_ptr
    if (ground_filter_mode_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter_ground;
        statFilter_ground.setInputCloud(ground_cloud_ptr);
        statFilter_ground.setMeanK(ground_meank_);
        statFilter_ground.setStddevMulThresh(ground_stdmul_);
        statFilter_ground.filter(*filtered_ground_cloud_ptr);
    }
    else
    {
        filtered_ground_cloud_ptr = ground_cloud_ptr;
    }
    
    //for no_ground_cloud_ptr
    if (no_ground_filter_mode_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter_no_ground;
        statFilter_no_ground.setInputCloud(no_ground_cloud_ptr);
        statFilter_no_ground.setMeanK(no_ground_meank_);
        statFilter_no_ground.setStddevMulThresh(no_ground_stdmul_);
        statFilter_no_ground.filter(*filtered_no_ground_cloud_ptr);
    }
    else
    {
        filtered_no_ground_cloud_ptr = no_ground_cloud_ptr;
    }
    
    if (show_points_size_)
    {
        std::cout<<"points_size of filtered_ground_cloud_ptr:"<<filtered_ground_cloud_ptr->points.size()<<std::endl;
        std::cout<<"points_size of filtered_no_ground_cloud_ptr:"<<filtered_no_ground_cloud_ptr->points.size()<<std::endl;
        std::cout<<std::endl;
    }

    publish_cloud(pub_pointcloud_ground, filtered_ground_cloud_ptr, in_cloud_ptr->header);
    publish_cloud(pub_pointcloud_no_ground, filtered_no_ground_cloud_ptr, in_cloud_ptr->header);
}


