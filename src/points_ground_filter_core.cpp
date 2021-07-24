#include "points_ground_filter_core.h"

PointsGroundFilter::PointsGroundFilter(ros::NodeHandle& nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/pandar_points_processed");
    nh.param<std::string>("pub_ground_topic", pub_ground_topic_, "/pandar_points_ground");
    nh.param<std::string>("pub_no_ground_topic", pub_no_ground_topic_, "/pandar_points_no_ground");

    nh.param<bool>("show_points_size", show_points_size_, "false");
    nh.param<bool>("show_time", show_time_, "false");
    
    nh.param<bool>("crop_range_mode", crop_range_mode_, "false");
    nh.param<float>("range_front", range_front_, 50);
    nh.param<float>("range_rear", range_rear_, 50);
    nh.param<float>("range_left", range_left_, 15);
    nh.param<float>("range_right", range_right_, 15);
    
    nh.param<float>("sensor_height", sensor_height_, 2.0);
    nh.param<float>("seeds_distance_threshold", seeds_distance_threshold_, 1.0);
    nh.param<float>("ground_distance_threshold", ground_distance_threshold_, 0.2);
    nh.param<int>("num_lpr", num_lpr_, 100);
    
    nh.param<int>("seg_num_front", seg_num_front_, 3);
    nh.param<int>("seg_num_rear", seg_num_rear_, 3);
    ros::param::get("~seg_distance_front", seg_distance_front_);
    ros::param::get("~seg_distance_rear", seg_distance_rear_);

    sub_point_cloud_ = nh.subscribe(sub_topic_, 1, &PointsGroundFilter::callback, this);
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(pub_ground_topic_, 1);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(pub_no_ground_topic_, 1);

    ros::spin();
}

PointsGroundFilter::~PointsGroundFilter()
{
}

bool compareZ(pgf::PointXYZIR a, pgf::PointXYZIR b)
{
    return a.z < b.z;
}

/*
    @brief Estimate the plane model.
    The ground plane model is: ax + by + cz + d = 0.
    Here the normal vector of the plane is: [a, b, c].
    The main step is performed SVD(UAV) on covariance matrix.
    Taking the sigular vector in U matrix according to the smallest
    sigular value in A, as the `normal`. `d` is then calculated
    according to mean ground points.
    @param pc: point clouds to estimate plane
    @param normal: normal vector of the plane
    @param d: plane parameter
*/
void PointsGroundFilter::estimatePlane(const pcl::PointCloud<pgf::PointXYZIR>::Ptr pc,
                                       Eigen::MatrixXf& normal,
                                       float& d)
{
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*pc, cov, pc_mean);
    
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    
    // Use the least singular vector as normal.
    normal = (svd.matrixU().col(2));
    
    // Compute d_ by mean ground seeds value.
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    d = -(normal.transpose() * seeds_mean)(0, 0);
}

/*
    @brief Extract initial seeds of the given point clouds sorted segment.
    This function filter ground seeds points accoring to heigt.
    LPR means low point representative.
    @param pc_sorted: sorted point clouds
    @param pc_seeds: points of seeds
*/
void PointsGroundFilter::extractInitialSeeds(const pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_sorted,
                                             pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_seeds)
{
    // Calculate the mean height value.
    float sum = 0;
    int cnt_lpr = 0;
    for(int i = 0; i < pc_sorted->points.size() && cnt_lpr < num_lpr_; i++)
    {
        sum += pc_sorted->points[i].z;
        cnt_lpr++;
    }
    float lpr_height = cnt_lpr != 0 ? sum / cnt_lpr : 0; // in case divide by 0
    
    int it = 0;
    for(int i = 0; i < pc_sorted->points.size(); i++)
    {
        if(pc_sorted->points[i].z < lpr_height + seeds_distance_threshold_) it++;
        else break;
    }
    int interval = floor(it / num_lpr_);
    
    // Filter point clouds those height is less than lpr.height + seeds_distance_threshold_.
    int cnt_seeds = 0;
    pc_seeds->clear();
    for(int i = 0; i < pc_sorted->points.size() && cnt_seeds < num_lpr_; i += interval)
    {
        pc_seeds->points.push_back(pc_sorted->points[i]);
        cnt_seeds++;
    }
}

void PointsGroundFilter::classifyPointCloudInSegment(pcl::PointCloud<pgf::PointXYZIR>::Ptr pc,
                                                     pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_ground,
                                                     pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_no_ground)
{
    // 1.Sort point clouds on Z-axis value.
    sort(pc->points.begin(), pc->end(), compareZ);
    
    // 2.Removal error point.
    // As there are some error mirror reflection under the ground, here regardless point under -1.5 * sensor_height_.
    // Sort point according to height, here uses z-axis in default.
    pcl::PointCloud<pgf::PointXYZIR>::iterator it = pc->points.begin();
    for(int i = 0; i < pc->points.size(); i++)
    {
        if(pc->points[i].z < -1.5 * sensor_height_) it++;
        else break;
    }
    pc->points.erase(pc->points.begin(), it);
    
    // 3.Extract initial ground seeds.
    pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_seeds(new pcl::PointCloud<pgf::PointXYZIR>);
    extractInitialSeeds(pc, pc_seeds);
    
    // 4.Estimate ground plane model.
    Eigen::MatrixXf normal;
    float d;
    estimatePlane(pc_seeds, normal, d);
    
    // 5.Filter point clouds by distance threshold, considering sqrt(a * a + b * b + c * c) = 1.
    // Now the pc->points.size() is less than the size of original point clouds.
    float a = normal(0);
    float b = normal(1);
    float c = normal(2);

    for(int r = 0; r < pc->points.size(); r++)
    {
        if(a * pc->points[r].x + b * pc->points[r].y + c * pc->points[r].z + d < ground_distance_threshold_)
        {
            pc_ground->points.push_back(pc->points[r]);
        }
        else
        {
            pc_no_ground->points.push_back(pc->points[r]);
        }
    }
}

void PointsGroundFilter::classifyPointCloud(pcl::PointCloud<pgf::PointXYZIR>::Ptr pc,
                                            pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_ground,
                                            pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_no_ground)
{
    
    // Create front segments in point clouds.
    std::vector<pcl::PointCloud<pgf::PointXYZIR>::Ptr> pc_array_front(seg_num_front_);
    for(int j = 0; j < pc_array_front.size(); j++)
    {
        pcl::PointCloud<pgf::PointXYZIR>::Ptr tmp(new pcl::PointCloud<pgf::PointXYZIR>);
        pc_array_front[j] = tmp;
    }
    
    // Create rear segments in point clouds.
    std::vector<pcl::PointCloud<pgf::PointXYZIR>::Ptr> pc_array_rear(seg_num_rear_);
    for(int j = 0; j < pc_array_rear.size(); j++)
    {
        pcl::PointCloud<pgf::PointXYZIR>::Ptr tmp(new pcl::PointCloud<pgf::PointXYZIR>);
        pc_array_rear[j] = tmp;
    }
    
    // Divide point clouds into segments.
    for(int i = 0; i < pc->points.size(); i++)
    {
        if(pc->points[i].x >= 0)
        {
            for(int j = 0; j < pc_array_front.size(); j++)
            {
                if(fabs(pc->points[i].x) < seg_distance_front_[j])
                {
                    pc_array_front[j]->points.push_back(pc->points[i]);
                    break;
                }
            }
        }
        else
        {
            for(int j = 0; j < pc_array_rear.size(); j++)
            {
                if(fabs(pc->points[i].x) < seg_distance_rear_[j])
                {
                    pc_array_rear[j]->points.push_back(pc->points[i]);
                    break;
                }
            }
        }
    }
    
    // Classify the ground points and no ground points in front segments.
    for(int j = 0; j < pc_array_front.size(); j++)
    {
        classifyPointCloudInSegment(pc_array_front[j], pc_ground, pc_no_ground);
    }
    
    // Classify the ground points and no ground points in rear segments.
    for(int j = 0; j < pc_array_rear.size(); j++)
    {
        classifyPointCloudInSegment(pc_array_rear[j], pc_ground, pc_no_ground);
    }
}

void PointsGroundFilter::cropPointCloud(const pcl::PointCloud<pgf::PointXYZIR>::Ptr pc,
                                        pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_cropped)
{
    for(int i = 0; i < pc->points.size(); i++)
    {
        if(pc->points[i].x < range_front_ && pc->points[i].x > -range_rear_ && pc->points[i].y < range_left_ && pc->points[i].y > -range_right_)
        {
            pc_cropped->points.push_back(pc->points[i]);
        }
    }
}

void PointsGroundFilter::callback(const sensor_msgs::PointCloud2ConstPtr pc_msg)
{
    ros::Time time_start = ros::Time::now();
    
    // Convert ROS msg to pcl form.
    pcl::PointCloud<pgf::PointXYZIR>::Ptr pc(new pcl::PointCloud<pgf::PointXYZIR>);
    pcl::fromROSMsg(*pc_msg, *pc);
    
    /*
    // Copy the original point clouds.
    pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_all(new pcl::PointCloud<pgf::PointXYZIR>);
    pgf::PointXYZIRL point;
    for(size_t i = 0; i < pc->points.size(); i++)
    {
        point.x = pc->points[i].x;
        point.y = pc->points[i].y;
        point.z = pc->points[i].z;
        point.label = 0u;
        point.intensity = pc->points[i].intensity;
        point.ring = pc->points[i].ring;
        pc_all->points.push_back(point);
    }
    */
    
    pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_cropped(new pcl::PointCloud<pgf::PointXYZIR>);
    if(crop_range_mode_)
    {
        cropPointCloud(pc, pc_cropped);
    }
    else
    {
        pc_cropped = pc;
    }
    
    // Classify the point clouds.
    pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_ground(new pcl::PointCloud<pgf::PointXYZIR>);
    pcl::PointCloud<pgf::PointXYZIR>::Ptr pc_no_ground(new pcl::PointCloud<pgf::PointXYZIR>);
    classifyPointCloud(pc_cropped, pc_ground, pc_no_ground);
    
    // Publish ground points.
    sensor_msgs::PointCloud2 pc_msg_ground;
    pcl::toROSMsg(*pc_ground, pc_msg_ground);
    pc_msg_ground.header.stamp = pc_msg->header.stamp;
    pc_msg_ground.header.frame_id = pc_msg->header.frame_id;
    pub_ground_.publish(pc_msg_ground);

    // Publish no ground points.
    sensor_msgs::PointCloud2 pc_msg_no_ground;
    pcl::toROSMsg(*pc_no_ground, pc_msg_no_ground);
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
        std::cout << "size of ground point clouds:" << pc_ground->points.size() << std::endl;
        std::cout << "size of no ground point clouds:" << pc_no_ground->points.size() << std::endl;
    }
    
    if(show_time_)
    {
        std::cout << "cost time:" << time_end - time_start << "s" << std::endl;
    }
}
