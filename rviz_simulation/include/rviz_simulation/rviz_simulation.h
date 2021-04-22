
#ifndef _RVIZ_SIMULATION_H_
#define _RVIZ_SIMULATION_H_
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>

namespace rviz_simulation
{

class Pose_sim
{
public:
    Pose_sim(ros::NodeHandle & nh, ros::NodeHandle nh_private);
    ~Pose_sim();

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher path_pub_;

    std::string ns;

    tf::TransformBroadcaster broadcaster;

    void Pose_sim_Cb(const geometry_msgs::Pose::ConstPtr& msg);

};

}

#endif
