#include "points_ground_filter_core.h"

PointsGroundFilter::PointsGroundFilter(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/rslidar_points");
    nh.param<std::string>("pub_ground_topic", pub_ground_topic_, "/rslidar_points_ground");
    nh.param<std::string>("pub_no_ground_topic", pub_no_ground_topic_, "/rslidar_points_no_ground");
    
    nh.param<bool>("show_points_size", show_points_size_, false);
    nh.param<bool>("show_time", show_time_, false);

    nh.param<float>("sensor_height", sensor_height_, 2.0);
    nh.param<float>("radius_divider", radius_divider_, 0.15);
    nh.param<float>("theta_divider", theta_divider_, 0.4);
    nh.param<float>("local_height_threshold", local_height_threshold_, 0.2);
    nh.param<float>("general_slope_threshold", general_slope_threshold_, 2);
    
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

void PointsGroundFilter::Spin(){}

void PointsGroundFilter::convert_XYZ_to_XYZRTColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
                                    std::vector<PointCloudXYZRTColor> &out_pc)
{
    //floor(x)返回小于或等于x的最大整数
    //ceil(x)返回大于x的最小整数
    size_t num = ceil(360 / theta_divider_);

    //out_pc中包含num条射线，每条射线都是PointXYZRTColor型点云
    out_pc.resize(num);

    //以射线的形式组织点云
    #pragma omp for
    for (size_t i = 0; i < in_pc_ptr->points.size(); i++)
    {
        PointXYZRTColor new_point;
        auto radius = (float)sqrt(in_pc_ptr->points[i].x * in_pc_ptr->points[i].x + in_pc_ptr->points[i].y * in_pc_ptr->points[i].y);
        auto theta = (float)atan2(in_pc_ptr->points[i].y, in_pc_ptr->points[i].x) * 180 / PI;
        if (theta < 0)
        {
            theta += 360;
        }

        //径向微分
        auto radius_idx = (size_t)floor(radius / radius_divider_);
        //周向微分
        auto theta_idx = (size_t)floor(theta / theta_divider_);

        new_point.point = in_pc_ptr->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radius_idx = radius_idx;
        new_point.theta_idx = theta_idx;
        new_point.original_idx = i;

        out_pc[theta_idx].push_back(new_point);
    }

    //将同一根射线上的点按照半径排序
    #pragma omp for
    for (size_t i = 0; i < num; i++)
    {
        std::sort(out_pc[i].begin(), out_pc[i].end(), [](const PointXYZRTColor &a, const PointXYZRTColor &b) {return a.radius < b.radius;});
    }
}

void PointsGroundFilter::classify_pc(std::vector<PointCloudXYZRTColor> &in_pc,
                                    pcl::PointIndices &ground_indices,
                                    pcl::PointIndices &no_ground_indices)
{
    ground_indices.indices.clear();
    no_ground_indices.indices.clear();
    
    //遍历每一条射线
    #pragma omp for
    for (size_t i = 0; i < in_pc.size(); i++)
    {
        float pre_radius = 0;
        float pre_z = - sensor_height_;
        bool pre_ground = true;
        
        //遍历射线上的每一个点
        for (size_t j = 0; j < in_pc[i].size(); j++) 
        {
            float cur_radius = in_pc[i][j].radius;
            float cur_z = in_pc[i][j].point.z;
            bool cur_ground;
            
            //当前点与前一个点的距离
            float distance = cur_radius - pre_radius;

            //局部高度阈值
            float loc_height_th = local_height_threshold_;
            if (j == 0) {loc_height_th = 2 * loc_height_th;}
            
            //abs(x)对int变量求绝对值
            //fabs(x)对float变量或double变量求绝对值
            
            //全局高度阈值
            float gen_height_th = fabs(cur_radius * general_slope_threshold_ * PI / 180) + loc_height_th;

            //若当前点与前一个点高度相近
            if (fabs(pre_z - cur_z) <= loc_height_th)
            {
                if (pre_ground) {cur_ground = true;}
                else
                {
                    //判断当前点与地面距离
                    if (fabs(- sensor_height_ - cur_z) <= gen_height_th) {cur_ground = true;}
                    else {cur_ground = false;}
                }
            }
            //若当前点与前一个点高度不相近
            else
            {
                if (pre_ground) {cur_ground = false;}
                else
                {
                    //判断当前点与地面距离
                    if (fabs(- sensor_height_ - cur_z) <= gen_height_th) {cur_ground = true;}
                    else {cur_ground = false;}
                }
            }

            //提取索引
            if (cur_ground)
            {
                ground_indices.indices.push_back(in_pc[i][j].original_idx);
            }
            else
            {
                no_ground_indices.indices.push_back(in_pc[i][j].original_idx);
            }

            pre_radius = cur_radius;
            pre_z = cur_z;
            pre_ground = cur_ground;
        }
    }
}

void PointsGroundFilter::publish_pc(const ros::Publisher &pub,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr,
                                    const std_msgs::Header &header)
{
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*pc_ptr, pc_msg);
    pc_msg.header = header;
    pub.publish(pc_msg);
}

void PointsGroundFilter::callback(const sensor_msgs::PointCloud2ConstPtr &in)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*in, *current_pc_ptr);

    ros::Time time_start = ros::Time::now();

    //组织点云
    std::vector<PointCloudXYZRTColor> organized_pc;
    convert_XYZ_to_XYZRTColor(current_pc_ptr, organized_pc);

    //判定地面点云和非地面点云
    pcl::PointIndices ground_indices, no_ground_indices;
    classify_pc(organized_pc, ground_indices, no_ground_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ground_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_no_ground_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractIndices<pcl::PointXYZ> extractor_ground;
    extractor_ground.setInputCloud(current_pc_ptr);
    extractor_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
    extractor_ground.setNegative(false); //true removes the indices, false leaves only the indices
    extractor_ground.filter(*ground_pc_ptr);

    pcl::ExtractIndices<pcl::PointXYZ> extractor_no_ground;
    extractor_no_ground.setInputCloud(current_pc_ptr);
    extractor_no_ground.setIndices(boost::make_shared<pcl::PointIndices>(no_ground_indices));
    extractor_no_ground.setNegative(false); //true removes the indices, false leaves only the indices
    extractor_no_ground.filter(*no_ground_pc_ptr);
    
    //针对ground_pc_ptr滤波
    if (ground_filter_mode_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter_ground;
        statFilter_ground.setInputCloud(ground_pc_ptr);
        statFilter_ground.setMeanK(ground_meank_);
        statFilter_ground.setStddevMulThresh(ground_stdmul_);
        statFilter_ground.filter(*filtered_ground_pc_ptr);
    }
    else {filtered_ground_pc_ptr = ground_pc_ptr;}
    
    //针对no_ground_pc_ptr滤波
    if (no_ground_filter_mode_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter_no_ground;
        statFilter_no_ground.setInputCloud(no_ground_pc_ptr);
        statFilter_no_ground.setMeanK(no_ground_meank_);
        statFilter_no_ground.setStddevMulThresh(no_ground_stdmul_);
        statFilter_no_ground.filter(*filtered_no_ground_pc_ptr);
    }
    else {filtered_no_ground_pc_ptr = no_ground_pc_ptr;}
    
    int size_ground = filtered_ground_pc_ptr->points.size();
    int size_no_ground = filtered_no_ground_pc_ptr->points.size();

    ros::Time time_end = ros::Time::now();

    if (show_points_size_ || show_time_)
    {
        std::cout<<""<<std::endl;
    }

    if (show_points_size_)
    {
        std::cout<<"size of ground point clouds:"<<size_ground<<std::endl;
        std::cout<<"size of no ground point clouds:"<<size_no_ground<<std::endl;
    }

    if (show_time_)
    {
        std::cout<<"cost time:"<<time_end - time_start<<"s"<<std::endl;
    }

    publish_pc(pub_ground_, filtered_ground_pc_ptr, in->header);
    publish_pc(pub_no_ground_, filtered_no_ground_pc_ptr, in->header);
}


