#include "points_ground_filter_core.h"

PointsGroundFilter::PointsGroundFilter(ros::NodeHandle& nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/rslidar_points");
    nh.param<std::string>("pub_ground_topic", pub_ground_topic_, "/rslidar_points_ground");
    nh.param<std::string>("pub_no_ground_topic", pub_no_ground_topic_, "/rslidar_points_no_ground");
    
    nh.param<bool>("show_points_size", show_points_size_, false);
    nh.param<bool>("show_time", show_time_, false);

    nh.param<float>("sensor_height", sensor_height_, 2.0);
    nh.param<float>("max_distance", max_distance_, 50.0);
    nh.param<float>("radius_divider", radius_divider_, 0.15);
    nh.param<float>("theta_divider", theta_divider_, 0.4);
    nh.param<float>("local_slope_threshold", local_slope_threshold_, 10);
    nh.param<float>("general_slope_threshold", general_slope_threshold_, 4);
    
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
    // floor(x)返回小于或等于x的最大整数
    // ceil(x)返回大于x的最小整数
    size_t num_theta = ceil(360 / theta_divider_);
    size_t num_radius = ceil(max_distance_ / radius_divider_);

    // sum_z_array_中包含num_theta条射线，每条射线包含num_radius个扇形单元格，每个单元格存储高度之和
    sum_z_array_.resize(num_theta);
    for(size_t i = 0; i < sum_z_array_.size(); i++)
    {
        sum_z_array_[i].resize(num_radius);
        for(size_t j = 0; j < sum_z_array_[i].size(); j++)
        {
            sum_z_array_[i][j] = 0.0;
        }
    }
    
    // num_array_中包含num_theta条射线，每条射线包含num_radius个扇形单元格，每个单元格存储点云数量
    num_array_.resize(num_theta);
    for(size_t i = 0; i < num_array_.size(); i++)
    {
        num_array_[i].resize(num_radius);
        for(size_t j = 0; j < num_array_[i].size(); j++)
        {
            num_array_[i][j] = 0;
        }
    }
    
    // label_array_中包含num_theta条射线，每条射线包含num_radius个扇形单元格，每个单元格存储标签值，0为地面，1非地面
    label_array_.resize(num_theta);
    for(size_t i = 0; i < label_array_.size(); i++)
    {
        label_array_[i].resize(num_radius);
        for(size_t j = 0; j < label_array_[i].size(); j++)
        {
            label_array_[i][j] = 0;
        }
    }

    // 以扇形单元格的形式组织点云
    #pragma omp for
    for(size_t r = 0; r < pc->points.size(); r++)
    {
        float radius = sqrt(pc->points[r].x * pc->points[r].x + pc->points[r].y * pc->points[r].y);
        float theta = atan2(pc->points[r].y, pc->points[r].x) * 180 / PI;
        if(theta < 0)
        {
            theta += 360;
        }

        size_t radius_idx = floor(radius / radius_divider_);
        size_t theta_idx = floor(theta / theta_divider_);
        
        pc->points[r].radius_idx = radius_idx;
        pc->points[r].theta_idx = theta_idx;
        
        sum_z_array_[theta_idx][radius_idx] += pc->points[r].z;
        num_array_[theta_idx][radius_idx] += 1;
    }
}

void PointsGroundFilter::classifyPointCloud(const pcl::PointCloud<pgf::PointXYZICustom>::Ptr pc,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_no_ground)
{
    // 遍历每一条射线
    #pragma omp for
    for(size_t i = 0; i < num_array_.size(); i++)
    {
        float pre_radius;
        float pre_z;
        bool pre_ground;
        bool initialized = false;
        
        // 遍历射线上的每一个扇形单元格
        for(size_t j = 0; j < num_array_[i].size(); j++) 
        {
            float cur_radius = radius_divider_ * j;
            float cur_z;
            bool cur_ground;
            
            if(num_array_[i][j] == 0) {continue;}

            cur_z = sum_z_array_[i][j] / num_array_[i][j];

            // abs(x)对int变量求绝对值
            // fabs(x)对float变量或double变量求绝对值
            // atan(x)表示x的反正切，其返回值为[-pi/2, +pi/2]之间的一个数
            // atan2(y, x)表示y / x的反正切，其返回值为[-pi, +pi]之间的一个数
            
            if(!initialized)
            {
                // 根据全局坡度判定
                float slope_g = atan2(fabs(cur_z - (- sensor_height_)), cur_radius) * 180 / PI;
                if(slope_g <= general_slope_threshold_) {cur_ground = true;}
                else {cur_ground = false;}
                
                initialized = true;
            }
            else
            {
                // 根据局部坡度判定
                float slope_l = atan2(fabs(cur_z - pre_z), (cur_radius - pre_radius)) * 180 / PI;
                if(slope_l <= local_slope_threshold_)
                {
                    if(pre_ground) {cur_ground = true;}
                    else
                    {
                        // 根据全局坡度判定
                        float slope_g = atan2(fabs(cur_z - (- sensor_height_)), cur_radius) * 180 / PI;
                        if(slope_g <= general_slope_threshold_) {cur_ground = true;}
                        else {cur_ground = false;}
                    }
                }
                else {cur_ground = false;}
            }
            
            if(cur_ground) {label_array_[i][j] = 0;}
            else {label_array_[i][j] = 1;}

            pre_radius = cur_radius;
            pre_z = cur_z;
            pre_ground = cur_ground;
        }
    }
    
    size_t num_theta = ceil(360 / theta_divider_);
    size_t num_radius = ceil(max_distance_ / radius_divider_);
    
    // 遍历每一个点
    #pragma omp for
    for(size_t r = 0; r < pc->points.size(); r++)
    {
        size_t theta_idx = pc->points[r].theta_idx;
        size_t radius_idx = pc->points[r].radius_idx;
        if(theta_idx < num_theta && radius_idx < num_radius)
        {
            pcl::PointXYZI point;
            point.x = pc->points[r].x;
            point.y = pc->points[r].y;
            point.z = pc->points[r].z;
            point.intensity = pc->points[r].intensity;
            
            if(label_array_[theta_idx][radius_idx] == 0)
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


