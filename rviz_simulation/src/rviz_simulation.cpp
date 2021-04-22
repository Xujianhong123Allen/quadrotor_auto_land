/*****************************
   Author: Jianhong Xu

   Date: 2021年01月18日 星期一 09时39分06秒 

   Description: 根据pose在rviz中进行仿真

******************************/

#include <rviz_simulation/rviz_simulation.h>



namespace rviz_simulation
{

    Pose_sim::Pose_sim(ros::NodeHandle &nh, ros::NodeHandle nh_private)
    {

        pose_sub_ = nh.subscribe("/quadrotor/pose",1,&Pose_sim::Pose_sim_Cb,this);
        path_pub_ = nh.advertise<nav_msgs::Path>("/quadrotor/trajectory",1) ;

    }

    Pose_sim::~Pose_sim()
    {
        std::cout<<"Quadrotor pose rviz simulation node quit!"<<std::endl;
    }

    void Pose_sim::Pose_sim_Cb(const  geometry_msgs::Pose::ConstPtr& msg)
    {
//         发布路径path的msgs
        static ros::Time current_time, last_time;
        static nav_msgs::Path path;
        static geometry_msgs::PoseStamped this_pose_stamped;
        current_time = ros::Time::now();
        this_pose_stamped.pose.orientation.x=msg->orientation.x;
        this_pose_stamped.pose.orientation.y=msg->orientation.y;
        this_pose_stamped.pose.orientation.z=msg->orientation.z;
        this_pose_stamped.pose.orientation.w=msg->orientation.w;

        this_pose_stamped.pose.position.x=msg->position.x/1000;
        this_pose_stamped.pose.position.y=msg->position.y/1000;
        this_pose_stamped.pose.position.z=msg->position.z/1000;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="odom";
        path.poses.push_back(this_pose_stamped);

        path_pub_.publish(path);

        last_time=current_time;

        ROS_INFO("Quaternion(x,y,z,w):%f %f %f %f",msg->orientation.x,
            msg->orientation.y,msg->orientation.z,msg->orientation.w);
        ROS_INFO("position(x, y, z):%f,%f,%f",msg->position.x,msg->position.y,msg->position.z);

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z,msg->orientation.w), tf::Vector3(
                        msg->position.x/1000,msg->position.y/1000,msg->position.z/1000)),
                ros::Time::now(),"world", "quadrotor"));

    }
}
