#include "points_ground_filter_core.h"

PointsGroundFilter::PointsGroundFilter(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/rslidar_points");
    nh.param<std::string>("pub_ground_topic", pub_ground_topic_, "/rslidar_points_ground");
    nh.param<std::string>("pub_no_ground_topic", pub_no_ground_topic_, "/rslidar_points_no_ground");
    nh.param<std::string>("pub_marker_topic", pub_marker_topic_, "/feasible_region");
    
    nh.param<bool>("show_points_size", show_points_size_, false);
    nh.param<bool>("show_time", show_time_, false);

    nh.param<float>("sensor_height", sensor_height_, 2.0);
    nh.param<float>("radius_divider", radius_divider_, 0.15);
    nh.param<float>("theta_divider", theta_divider_, 0.4);
    nh.param<float>("local_slope_threshold", local_slope_threshold_, 10);
    nh.param<float>("general_slope_threshold", general_slope_threshold_, 4);
    nh.param<float>("curb_height_threshold", curb_height_threshold_, 0.1);
    
    nh.param<bool>("ground_filter_mode", ground_filter_mode_, false);
    nh.param<float>("ground_meank", ground_meank_, 5);
    nh.param<float>("ground_stdmul", ground_stdmul_, 1);
    
    nh.param<bool>("no_ground_filter_mode", no_ground_filter_mode_, false);
    nh.param<float>("no_ground_meank", no_ground_meank_, 5);
    nh.param<float>("no_ground_stdmul", no_ground_stdmul_, 1);
    
    sub_ = nh.subscribe(sub_topic_, 1, &PointsGroundFilter::callback, this);
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(pub_ground_topic_, 1);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(pub_no_ground_topic_, 1);
    pub_marker_ = nh.advertise<visualization_msgs::Marker>(pub_marker_topic_, 1);
    
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

    //将同一条射线上的点按照半径排序
    #pragma omp for
    for (size_t i = 0; i < num; i++)
    {
        std::sort(out_pc[i].begin(), out_pc[i].end(), [](const PointXYZRTColor &a, const PointXYZRTColor &b) {return a.radius < b.radius;});
    }
}

void PointsGroundFilter::classify_pc(std::vector<PointCloudXYZRTColor> &in_pc,
                                    pcl::PointIndices &freespace_indices,
                                    pcl::PointIndices &obstacle_indices)
{
    freespace_indices.indices.clear();
    obstacle_indices.indices.clear();
    
    //遍历每一条射线
    #pragma omp for
    for (size_t i = 0; i < in_pc.size(); i++)
    {
        float pre_radius = 0;
        float pre_z = - sensor_height_;
        bool pre_ground = true;
        
        //遍历射线上的每一个点，分割地面的点与障碍物的点
        for (size_t j = 0; j < in_pc[i].size(); j++) 
        {
            float cur_radius = in_pc[i][j].radius;
            float cur_z = in_pc[i][j].point.z;
            bool cur_ground;

            //abs(x)对int变量求绝对值
            //fabs(x)对float变量或double变量求绝对值
            //atan(x)表示x的反正切，其返回值为[-pi/2, +pi/2]之间的一个数
            //atan2(y, x)表示y / x的反正切，其返回值为[-pi, +pi]之间的一个数
            
            //如果当前点与上一个点距离很近，则舍弃当前点，相当于体素栅格滤波
            if (fabs(cur_z - pre_z) <= 0.15 && fabs(cur_radius - pre_radius) <= 0.15) {continue;}
            
            if (j == 0)
            {
                //根据全局坡度判定
                float slope_g = (float)(atan2(fabs(cur_z - (- sensor_height_)), cur_radius) * 180 / PI);
                if (slope_g <= general_slope_threshold_) {cur_ground = true;}
                else {cur_ground = false;}
            }
            else
            {
                //根据局部坡度判定
                float slope_l = (float)(atan2(fabs(cur_z - pre_z), (cur_radius - pre_radius)) * 180 / PI);
                if (slope_l <= local_slope_threshold_)
                {
                    if (pre_ground) {cur_ground = true;}
                    else
                    {
                        //根据全局坡度判定
                        float slope_g = (float)(atan2(fabs(cur_z - (- sensor_height_)), cur_radius) * 180 / PI);
                        if (slope_g <= general_slope_threshold_) {cur_ground = true;}
                        else {cur_ground = false;}
                    }
                }
                else {cur_ground = false;}
            }
            if (!cur_ground) {obstacle_indices.indices.push_back(in_pc[i][j].original_idx);}

            pre_radius = cur_radius;
            pre_z = cur_z;
            pre_ground = cur_ground;
        }

        pre_radius = 0;
        pre_z = - sensor_height_;
        bool pre_freespace = true;
        
        //遍历射线上的每一个点，提取可行域
        for (size_t j = 0; j < in_pc[i].size(); j++) 
        {
            float cur_radius = in_pc[i][j].radius;
            float cur_z = in_pc[i][j].point.z;
            bool cur_freespace;

            //如果当前点与上一个点距离很近，则舍弃当前点，相当于体素栅格滤波
            if (fabs(cur_z - pre_z) <= 0.15 && fabs(cur_radius - pre_radius) <= 0.15) {continue;}
            
            if (j == 0)
            {
                //根据全局坡度判定
                float slope_g = (float)(atan2(fabs(cur_z - (- sensor_height_)), cur_radius) * 180 / PI);
                if (slope_g <= general_slope_threshold_) {cur_freespace = true;}
                else {cur_freespace = false;}
            }
            else
            {
                //根据高度差判定
                if ((cur_z - pre_z) <= curb_height_threshold_ && pre_freespace) {cur_freespace = true;}
                else {cur_freespace = false;}
            }
            if (pre_freespace) {freespace_indices.indices.push_back(in_pc[i][j].original_idx);}

            pre_radius = cur_radius;
            pre_z = cur_z;
            pre_freespace = cur_freespace;
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

void PointsGroundFilter::publish_marker(const ros::Publisher &pub,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
                                        visualization_msgs::Marker &region,
                                        std_msgs::Header header)
{
    std::vector<PointR> pc;
    size_t num = ceil(360 / theta_divider_);

    //初始化PointXYZRTColor型数组，元素的索引由极角决定
    #pragma omp for
    for (size_t p = 0; p < num; p++)
    {
        PointR new_point;
        new_point.radius = 0;
        new_point.point.x = 0;
        new_point.point.y = 0;
        new_point.point.z = - sensor_height_;
        pc.push_back(new_point);
    }

    //将最远点作为数组中每个极角对应的元素
    #pragma omp for
    for (size_t i = 0; i < in_pc_ptr->points.size(); i++)
    {
        auto radius = (float)sqrt(in_pc_ptr->points[i].x * in_pc_ptr->points[i].x + in_pc_ptr->points[i].y * in_pc_ptr->points[i].y);
        auto theta = (float)atan2(in_pc_ptr->points[i].y, in_pc_ptr->points[i].x) * 180 / PI;
        if (theta < 0) {theta += 360;}

        auto theta_idx = (size_t)floor(theta / theta_divider_);

        if (radius > pc[theta_idx].radius)
        {
            pc[theta_idx].radius = radius;
            pc[theta_idx].point.x = in_pc_ptr->points[i].x;
            pc[theta_idx].point.y = in_pc_ptr->points[i].y;
            pc[theta_idx].point.z = - sensor_height_;
        }
    }

    //数据腐蚀
    for (size_t iter = 0; iter < 1; iter++)
    {
        std::vector<PointR> pc_post;
        for (size_t p = 0; p < num; p++)
        {
            size_t n = 2;
            std::vector<float> radii;
            radii.resize(2 * n + 1);
            radii[n] = pc[p].radius;
            for (size_t k = 0; k < n; k++)
            {
                radii[n + k + 1] = pc[(p + k + 1) % num].radius;
                radii[n - k - 1] = pc[(p - k - 1 + num) % num].radius;
            }
            
            float radiusm = (float)DBL_MAX;
            for (size_t k = 0; k < 2 * n + 1; k++) {if (radii[k] < radiusm) {radiusm = radii[k];}}

            PointR new_point;
            new_point.radius = radiusm;
            new_point.point.x = radiusm * cos(p * theta_divider_ * PI / 180);
            new_point.point.y = radiusm * sin(p * theta_divider_ * PI / 180);
            new_point.point.z = - sensor_height_;
            pc_post.push_back(new_point);
        }
        pc = pc_post;
    }

    //利用最小可通行宽度筛选
    for (size_t iter = 0; iter < 4; iter++)
    {
        std::vector<PointR> pc_post;
        for (size_t p = 0; p < num; p++)
        {
            float passable_width = 1.5;
            float tolerance = 10.0 / (1 + iter);
            bool overflow = false;
            bool flag = false;
            
            //设置搜索范围，单位度
            float search_range = 10;

            //正向搜索
            float radius_f;
            size_t sum_f;
            for (size_t k = 1; k < floor(search_range / theta_divider_); k++)
            {
                if (pc[(p + k) % num].radius < pc[p].radius - tolerance)
                {
                    radius_f = pc[(p + k) % num].radius;
                    sum_f = k;
                    break;
                }
                if (k == floor(search_range / theta_divider_) - 1) {overflow = true;}
            }
            //反向搜索
            float radius_b;
            size_t sum_b;
            for (size_t k = 1; k < floor(search_range / theta_divider_); k++)
            {
                if (pc[(p - k + num) % num].radius < pc[p].radius - tolerance)
                {
                    radius_b = pc[(p - k + num) % num].radius;
                    sum_b = k;
                    break;
                }
                if (k == floor(search_range / theta_divider_) - 1) {overflow = true;}
            }

            //判定是否能够通行
            if (overflow) {flag = true;}
            else
            {
                float r = (radius_f > radius_b) ? radius_f : radius_b;
                float a = (sum_f + sum_b) * theta_divider_ * PI / 180;
                if (r * a > passable_width) {flag = true;}
            }

            float radius;
            if (flag) {radius = pc[p].radius;} else {radius = (radius_f > radius_b) ? radius_f : radius_b;}
            
            PointR new_point;
            new_point.radius = radius;
            new_point.point.x = radius * cos(p * theta_divider_ * PI / 180);
            new_point.point.y = radius * sin(p * theta_divider_ * PI / 180);
            new_point.point.z = - sensor_height_;
            pc_post.push_back(new_point);
        }
        pc = pc_post;
    }
    
    geometry_msgs::Point origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = - sensor_height_;

    //用连续三角形表示可行域，计算三角形的顶点
    for (size_t p = 0; p < num; p++)
    {
        region.points.push_back(origin);
        region.points.push_back(pc[p].point);
        region.points.push_back(pc[(p + 1) % num].point);
    }

    region.header = header;

    //设置该标记的命名空间和ID，ID应该是独一无二的
    //具有相同命名空间和ID的标记将会覆盖前一个
    region.ns = "feasible_region";
    region.id = 0;
    
    //设置标记类型
    region.type = visualization_msgs::Marker::TRIANGLE_LIST;
    
    //设置标记行为：ADD为添加，DELETE为删除
    region.action = visualization_msgs::Marker::ADD;

    //设置标记尺寸
    region.scale.x = 1;
    region.scale.y = 1;
    region.scale.z = 1;

    //设置标记颜色，确保不透明度alpha不为0
    region.color.r = 0.0f;
    region.color.g = 0.8f;
    region.color.b = 0.0f;
    region.color.a = 0.65;

    region.lifetime = ros::Duration(0.1);
    region.text = ' ';

    pub_marker_.publish(region);
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

    visualization_msgs::Marker region;
    publish_pc(pub_ground_, filtered_ground_pc_ptr, in->header);
    publish_pc(pub_no_ground_, filtered_no_ground_pc_ptr, in->header);
    publish_marker(pub_marker_, filtered_ground_pc_ptr, region, in->header);

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
}


