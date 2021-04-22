
#include <rviz_simulation/rviz_simulation.h> 

int main(int argc,char * argv[])
{
    ros::init(argc,argv,"rviz_simulation");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    rviz_simulation::Pose_sim enerator(nh,nh_private);
    ros::spin();
    ros::shutdown();
    return 0;
}
