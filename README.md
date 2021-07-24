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
   <param name="sub_topic" value="/pandar_points_processed"/>
   <param name="pub_ground_topic" value="/pandar_points_ground"/>
   <param name="pub_no_ground_topic" value="/pandar_points_no_ground"/>
        
   <param name="show_points_size" value="true"/>
   <param name="show_time" value="true"/>
        
   <param name="crop_range_mode" value="true"/>
   <param name="range_front" value="50"/>
   <param name="range_rear" value="50"/>
   <param name="range_left" value="15"/>
   <param name="range_right" value="15"/>
        
   <param name="sensor_height" value="2.0"/>
   <param name="seeds_distance_threshold" value="1.0"/>
   <param name="ground_distance_threshold" value="0.2"/>
   <param name="num_lpr" value="100"/>
        
   <param name="seg_num_front" value="5"/>
   <param name="seg_num_rear" value="5"/>
   <rosparam param="seg_distance_front" > [10, 20, 30, 40, 50] </rosparam>
   <rosparam param="seg_distance_rear" > [10, 20, 30, 40, 50] </rosparam>
   ```
    - `sub_topic`指明订阅的点云话题。
    - `pub_ground_topic`指明发布的只包含地面点云的点云话题。
    - `pub_no_ground_topic`指明发布的不包含地面点云的点云话题。
    - `sensor_height`为传感器距地面高度，单位为米。
    - `seeds_distance_threshold`为选择种子点云的高度阈值，单位为米。
    - `ground_distance_threshold`为地面点云与拟合平面的距离阈值，单位为米。
    - `num_lpr`为种子点云数量。

## 运行
 - 启动`points_ground_filter`
   ```Shell
   roslaunch points_ground_filter points_ground_filter.launch
   ```

