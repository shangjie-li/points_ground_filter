#include "points_ground_filter_core.h"

PointsGroundFilter::PointsGroundFilter(ros::NodeHandle& nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/rslidar_points");
    nh.param<std::string>("pub_ground_topic", pub_ground_topic_, "/rslidar_points_ground");
    nh.param<std::string>("pub_no_ground_topic", pub_no_ground_topic_, "/rslidar_points_no_ground");
    
    nh.param<bool>("show_points_size", show_points_size_, false);
    nh.param<bool>("show_time", show_time_, false);

    nh.param<float>("max_x", max_x_, 50.0);
    nh.param<float>("max_y", max_y_, 15.0);
    nh.param<float>("x_divider", x_divider_, 0.15);
    nh.param<float>("y_divider", y_divider_, 0.15);
    nh.param<float>("local_slope_threshold", local_slope_threshold_, 10);
    
    nh.param<bool>("ground_filter_mode", ground_filter_mode_, false);
    nh.param<float>("ground_meank", ground_meank_, 5);
    nh.param<float>("ground_stdmul", ground_stdmul_, 1);
    
    nh.param<bool>("no_ground_filter_mode", no_ground_filter_mode_, false);
    nh.param<float>("no_ground_meank", no_ground_meank_, 5);
    nh.param<float>("no_ground_stdmul", no_ground_stdmul_, 1);
    
    sub_ = nh.subscribe(sub_topic_, 1, &PointsGroundFilter::callback, this);
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(pub_ground_topic_, 1);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(pub_no_ground_topic_, 1);
    
    ros::spin();
}

PointsGroundFilter::~PointsGroundFilter()
{
}

void PointsGroundFilter::convertPointCloud(pcl::PointCloud<pgf::PointXYZICustom>::Ptr pc)
{
    int nx = ceil(max_x_ / x_divider_);
    int ny = ceil(max_y_ / y_divider_);
    int cols = 2 * nx;
    int rows = 2 * ny;
    
    height_mat_ = cv::Mat::zeros(rows, cols, CV_32FC1);
    num_mat_ = cv::Mat::zeros(rows, cols, CV_32FC1);
    slope_mat_ = cv::Mat::zeros(rows, cols, CV_32FC1);
    
    // Compute sum height in cells.
    #pragma omp for
    for(int r = 0; r < pc->points.size(); r++)
    {
        int x_idx = floor(pc->points[r].x / x_divider_);
        int y_idx = floor(pc->points[r].y / y_divider_);
        
        pc->points[r].x_idx = x_idx;
        pc->points[r].y_idx = y_idx;

        int col = nx + x_idx;
        int row = ny - y_idx;
        
        if(row >= 0 && row < rows && col >=0 && col < cols)
        {
            height_mat_.at<float>(row, col) += pc->points[r].z;
            num_mat_.at<float>(row, col) += 1;
        }
    }
}

void PointsGroundFilter::classifyPointCloud(const pcl::PointCloud<pgf::PointXYZICustom>::Ptr pc,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_no_ground)
{
    int nx = ceil(max_x_ / x_divider_);
    int ny = ceil(max_y_ / y_divider_);
    int cols = 2 * nx;
    int rows = 2 * ny;
    
    // Compute mean height in cells.
    height_mat_ /= num_mat_;
    
    // Compute local slope.
    #pragma omp for
    for(int i = 1; i < rows - 1; i++)
    {
        for(int j = 1; j < cols - 1; j++) 
        {
            if(num_mat_.at<float>(i, j) == 0) {continue;}
            else
            {
                float z1, z2, slope;
                float max_slope = 0;
                z1 = height_mat_.at<float>(i, j);
                
                if(num_mat_.at<float>(i - 1, j) != 0)
                {
                    z2 = height_mat_.at<float>(i - 1, j);
                    slope = atan2(fabs(z1 - z2), y_divider_) * 180 / PI;
                    max_slope = slope > max_slope ? slope : max_slope;
                }
                if(num_mat_.at<float>(i + 1, j) != 0)
                {
                    z2 = height_mat_.at<float>(i + 1, j);
                    slope = atan2(fabs(z1 - z2), y_divider_) * 180 / PI;
                    max_slope = slope > max_slope ? slope : max_slope;
                }
                if(num_mat_.at<float>(i, j - 1) != 0)
                {
                    z2 = height_mat_.at<float>(i, j - 1);
                    slope = atan2(fabs(z1 - z2), x_divider_) * 180 / PI;
                    max_slope = slope > max_slope ? slope : max_slope;
                }
                if(num_mat_.at<float>(i, j + 1) != 0)
                {
                    z2 = height_mat_.at<float>(i, j + 1);
                    slope = atan2(fabs(z1 - z2), x_divider_) * 180 / PI;
                    max_slope = slope > max_slope ? slope : max_slope;
                }
                slope_mat_.at<float>(i, j) = max_slope;
            }
        }
    }
    
    cv::Mat tmp(rows, cols, CV_8UC1);
    cv::threshold(slope_mat_, tmp, local_slope_threshold_, 255, cv::THRESH_BINARY);
    
    cv::Mat labels, stats, centroids;
    tmp.convertTo(tmp, CV_8UC1);
    int nccomps = cv::connectedComponentsWithStats(tmp, labels, stats, centroids);
    
    // Classify ground points and no ground points.
    #pragma omp for
    for(int r = 0; r < pc->points.size(); r++)
    {
        int x_idx = pc->points[r].x_idx;
        int y_idx = pc->points[r].y_idx;
        
        int col = nx + x_idx;
        int row = ny - y_idx;
        
        if(row >= 0 && row < rows && col >=0 && col < cols)
        {
            pcl::PointXYZI point;
            point.x = pc->points[r].x;
            point.y = pc->points[r].y;
            point.z = pc->points[r].z;
            point.intensity = pc->points[r].intensity;
            
            // labels.type(): CV_32F
            // labels(row, col): 0 - background;
            //                   1, 2, ... - connected components (descend as size of area).
            if(labels.at<int>(row, col) == 0)
            {
                pc_ground->points.push_back(point);
            }
            else
            {
                pc_no_ground->points.push_back(point);
            }
        }
    }
}

void PointsGroundFilter::callback(const sensor_msgs::PointCloud2ConstPtr pc_msg)
{
    ros::Time time_start = ros::Time::now();
    
    pcl::PointCloud<pgf::PointXYZICustom>::Ptr pc_current(new pcl::PointCloud<pgf::PointXYZICustom>);
    pcl::fromROSMsg(*pc_msg, *pc_current);

    // 组织点云
    convertPointCloud(pc_current);
    
    // 判定地面点云和非地面点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_no_ground(new pcl::PointCloud<pcl::PointXYZI>);
    classifyPointCloud(pc_current, pc_ground, pc_no_ground);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_no_ground_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    // 针对pc_ground滤波
    if(ground_filter_mode_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statFilter_ground;
        statFilter_ground.setInputCloud(pc_ground);
        statFilter_ground.setMeanK(ground_meank_);
        statFilter_ground.setStddevMulThresh(ground_stdmul_);
        statFilter_ground.filter(*pc_ground_filtered);
    }
    else {pc_ground_filtered = pc_ground;}
    
    // 针对pc_no_ground滤波
    if(no_ground_filter_mode_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statFilter_no_ground;
        statFilter_no_ground.setInputCloud(pc_no_ground);
        statFilter_no_ground.setMeanK(no_ground_meank_);
        statFilter_no_ground.setStddevMulThresh(no_ground_stdmul_);
        statFilter_no_ground.filter(*pc_no_ground_filtered);
    }
    else {pc_no_ground_filtered = pc_no_ground;}

    // 发布地面点云
    sensor_msgs::PointCloud2 pc_msg_ground;
    pcl::toROSMsg(*pc_ground_filtered, pc_msg_ground);
    pc_msg_ground.header.stamp = pc_msg->header.stamp;
    pc_msg_ground.header.frame_id = pc_msg->header.frame_id;
    pub_ground_.publish(pc_msg_ground);
    
    // 发布非地面点云
    sensor_msgs::PointCloud2 pc_msg_no_ground;
    pcl::toROSMsg(*pc_no_ground_filtered, pc_msg_no_ground);
    pc_msg_no_ground.header.stamp = pc_msg->header.stamp;
    pc_msg_no_ground.header.frame_id = pc_msg->header.frame_id;
    pub_no_ground_.publish(pc_msg_no_ground);
    
    ros::Time time_end = ros::Time::now();

    if(show_points_size_ || show_time_)
    {
        std::cout << "" << std::endl;
        std::cout << "[points_ground_filter]" << std::endl;
    }
    
    if(show_points_size_)
    {
        std::cout << "Size of ground point clouds: " << pc_ground_filtered->points.size() << std::endl;
        std::cout << "Size of no ground point clouds: " << pc_no_ground_filtered->points.size() << std::endl;
    }

    if(show_time_)
    {
        std::cout << "Time cost per frame: " << time_end - time_start << "s" << std::endl;
    }
}


