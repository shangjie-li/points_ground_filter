# points_ground_filter

ROS package for filtering groud points

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/points_ground_filter.git
   cd ..
   catkin_make
   ```
   
## 参数配置
 - 修改`points_ground_filter/launch/points_ground_filter.launch`
   ```Shell
   <param name="sub_topic" value="/rslidar_points_processed"/>
   <param name="pub_ground_topic" value="/rslidar_points_ground"/>
   <param name="pub_no_ground_topic" value="/rslidar_points_no_ground"/>
        
   <param name="show_points_size" value="true"/>
        
   <param name="sensor_height" value="2.05"/>
   <param name="radial_divider_angle" value="0.2"/>
   <param name="concentric_divider_distance" value="0.1"/>
        
   <param name="local_threshold_ratio" value="0.01"/>
   <param name="min_local_threshold" value="0.05"/>
   <param name="general_threshold_ratio" value="0.02"/>
   <param name="min_general_threshold" value="0.15"/>
   <param name="reclass_distance" value="0.2"/>
   
   <param name="ground_filter_mode" value="true"/>
   <param name="ground_meank" value="10"/>
   <param name="ground_stdmul" value="1.0"/>
        
   <param name="no_ground_filter_mode" value="true"/>
   <param name="no_ground_meank" value="10"/>
   <param name="no_ground_stdmul" value="1.0"/>
   ```
    - `sub_topic`指明订阅的点云话题。
    - `pub_ground_topic`指明发布的只包含地面点云的点云话题。
    - `pub_no_ground_topic`指明发布的不包含地面点云的点云话题。
    - `sensor_height`指明传感器距地面高度，单位为米。
    - `radial_divider_angle`为激光雷达水平角分辨率，单位为度。
    - `concentric_divider_distance`为距离分割单元的长度，单位为米。
    - `local_threshold_ratio`为局部阈值比例。
    - `min_local_threshold`为两点之间高度差的阈值，单位为米。
    - `general_threshold_ratio`为整体阈值比例。
    - `min_general_threshold`为点云整体与地面距离的阈值，单位为米。

## 运行
 - 启动`points_ground_filter`
   ```Shell
   roslauch points_ground_filter points_ground_filter.launch
   ```

## 说明
 - 当激光雷达水平安装时，地面点云滤波效果较好。
