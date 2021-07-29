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

   <param name="max_x" value="50.0"/>
   <param name="max_y" value="15.0"/>
   <param name="x_divider" value="0.15"/>
   <param name="y_divider" value="0.15"/>
   <param name="local_slope_threshold" value="10"/>

   <param name="ground_filter_mode" value="false"/>
   <param name="ground_meank" value="5"/>
   <param name="ground_stdmul" value="1.0"/>

   <param name="no_ground_filter_mode" value="false"/>
   <param name="no_ground_meank" value="5"/>
   <param name="no_ground_stdmul" value="1.0"/>
   ```
    - `sub_topic`指明订阅的点云话题。
    - `pub_ground_topic`指明发布的只包含地面点云的点云话题。
    - `pub_no_ground_topic`指明发布的不包含地面点云的点云话题。
    - `max_x`为限制X坐标最大值，单位为米。
    - `max_y`为限制Y坐标最大值，单位为米。
    - `x_divider`为X坐标分割长度，单位为米。
    - `y_divider`为Y坐标分割长度，单位为米。
    - `local_slope_threshold`为局部坡度阈值，单位为度。

## 运行
 - 启动`points_ground_filter`
   ```Shell
   roslaunch points_ground_filter points_ground_filter.launch
   ```

## 说明
 - 当激光雷达水平安装时，地面点云滤波效果较好。
