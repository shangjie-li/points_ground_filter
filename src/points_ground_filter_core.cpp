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

PointsGroundFilter::~PointsGroundFilter(){}

void PointsGroundFilter::convertPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
                                           std::vector<std::vector<std::vector<PointXYZICustom>>>& pc_converted)
{
    // floor(x)返回小于或等于x的最大整数
    // ceil(x)返回大于x的最小整数
    size_t num_theta = ceil(360 / theta_divider_);
    size_t num_radius = ceil(max_distance_ / radius_divider_);

    // pc_converted中包含num_theta条射线，每条射线包含num_radius个扇形单元格
    // 每个单元格内存储PointXYZICustom类型点的集合
    pc_converted.resize(num_theta);
    for(size_t j = 0; j < pc_converted.size(); j++)
    {
        pc_converted[j].resize(num_radius);
    }

    // 以扇形单元格的形式组织点云
    #pragma omp for
    for(size_t i = 0; i < pc->points.size(); i++)
    {
        PointXYZICustom new_point;
        float radius = sqrt(pc->points[i].x * pc->points[i].x + pc->points[i].y * pc->points[i].y);
        float theta = atan2(pc->points[i].y, pc->points[i].x) * 180 / PI;
        if(theta < 0)
        {
            theta += 360;
        }

        size_t radius_idx = floor(radius / radius_divider_);
        size_t theta_idx = floor(theta / theta_divider_);

        new_point.point = pc->points[i];
        new_point.original_idx = i;

        if(radius_idx < num_radius) {pc_converted[theta_idx][radius_idx].push_back(new_point);}
    }
}

void PointsGroundFilter::classifyPointCloud(const std::vector<std::vector<std::vector<PointXYZICustom>>>& pc,
                                            pcl::PointIndices& ground_indices,
                                            pcl::PointIndices& no_ground_indices)
{
    ground_indices.indices.clear();
    no_ground_indices.indices.clear();
    
    // 遍历每一条射线
    #pragma omp for
    for(size_t i = 0; i < pc.size(); i++)
    {
        float pre_radius;
        float pre_z;
        bool pre_ground;
        bool initialized = false;
        
        // 遍历射线上的每一个扇形单元格
        for(size_t j = 0; j < pc[i].size(); j++) 
        {
            float cur_radius = radius_divider_ * j;
            float cur_z;
            bool cur_ground;
            
            if(pc[i][j].size() == 0) {continue;}
            
            float sum_z = 0;
            for(size_t r = 0; r < pc[i][j].size(); r++) {sum_z += pc[i][j][r].point.z;}
            cur_z = sum_z / pc[i][j].size();

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
            
            if(cur_ground)
            {
                for(size_t r = 0; r < pc[i][j].size(); r++)
                {
                    ground_indices.indices.push_back(pc[i][j][r].original_idx);
                }
            }
            else
            {
                for(size_t r = 0; r < pc[i][j].size(); r++)
                {
                    no_ground_indices.indices.push_back(pc[i][j][r].original_idx);
                }
            }

            pre_radius = cur_radius;
            pre_z = cur_z;
            pre_ground = cur_ground;
        }
    }
}

void PointsGroundFilter::callback(const sensor_msgs::PointCloud2ConstPtr pc_msg)
{
    ros::Time time_start = ros::Time::now();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_current(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc_msg, *pc_current);

    // 组织点云
    std::vector<std::vector<std::vector<PointXYZICustom>>> pc_organized;
    convertPointCloud(pc_current, pc_organized);

    // 判定地面点云和非地面点云
    pcl::PointIndices ground_indices, no_ground_indices;
    classifyPointCloud(pc_organized, ground_indices, no_ground_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_no_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_no_ground_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ExtractIndices<pcl::PointXYZI> extractor_ground;
    extractor_ground.setInputCloud(pc_current);
    extractor_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
    extractor_ground.setNegative(false); // true removes the indices, false leaves only the indices
    extractor_ground.filter(*pc_ground);

    pcl::ExtractIndices<pcl::PointXYZI> extractor_no_ground;
    extractor_no_ground.setInputCloud(pc_current);
    extractor_no_ground.setIndices(boost::make_shared<pcl::PointIndices>(no_ground_indices));
    extractor_no_ground.setNegative(false); // true removes the indices, false leaves only the indices
    extractor_no_ground.filter(*pc_no_ground);
    
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
    }
    
    if(show_points_size_)
    {
        std::cout << "size of ground point clouds:" << pc_ground_filtered->points.size() << std::endl;
        std::cout << "size of no ground point clouds:" << pc_no_ground_filtered->points.size() << std::endl;
    }

    if(show_time_)
    {
        std::cout << "cost time:" << time_end - time_start << "s" << std::endl;
    }
}


