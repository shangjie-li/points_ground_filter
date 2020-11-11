#include "points_ground_filter_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_ground_filter");
    
    ros::NodeHandle nh("~");
    
    PointsGroundFilter core(nh);
    
    return 0;
}
