/**********************************************************
 * Author        : Jianhong XU
 * Email         : jianhongxu1006@gmail.com
 * Last modified : 2021年04月22日 星期四 21时33分46秒 
 * Description   : PX4在offboard模式下识别目标并自主降落
**********************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

mavros_msgs::State current_state;

double run_circle_theta = 0;
bool find_mark = false;
const double PI = 3.141592653;
int num = 0;
geometry_msgs::Pose estimate_pose;
geometry_msgs::Pose previous_pose;
bool value_pose = true;  //判断估计位姿态是否赋值

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void run_circle(geometry_msgs::PoseStamped &pose)
{
    if (num <= 400)
    {
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
        num++;
    }

    // 飞行至半径为1的圆上
    if (run_circle_theta <= 2 * PI)
    {
        if (fabs(pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y - 1) > 0.1)
        {
            pose.pose.position.x += 0.01;
        }
        else
        {
            pose.pose.position.x = cos(run_circle_theta);
            pose.pose.position.y = sin(run_circle_theta);
            run_circle_theta += PI * 0.004;
        }
        // if (run_circle_theta >= 0.7854)
        // {
        //     find_mark = true;
        // }
    }
    // 若未发现降落目标，则飞行至半径为2的圆上
    else if (run_circle_theta <= 4 * PI)
    {
        if (fabs((pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y) - 4) > 0.1)
        {
            pose.pose.position.x += 0.01;
        }
        else
        {
            pose.pose.position.x = cos(run_circle_theta - 2 * PI) * 2;
            pose.pose.position.y = sin(run_circle_theta - 2 * PI) * 2;
            run_circle_theta += PI * 0.004;
        }
    }
    // // 若未发现降落目标，则飞行至半径为3的圆上
    // else
    // {
    //     if (fabs((pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y) - 9) > 0.1)
    //     {
    //         pose.pose.position.x += 0.01;
    //     }
    //     else
    //     {
    //         pose.pose.position.x = cos(run_circle_theta - 4 * PI) * 3;
    //         pose.pose.position.y = sin(run_circle_theta - 4 * PI) * 3;
    //         run_circle_theta += PI * 0.002;

    //         if (run_circle_theta >= 6 * PI)
    //         {
    //             run_circle_theta = 6 * PI;
    //         }
    //     }
    // }
    std::cout << "x: " << pose.pose.position.x << "\ty: " << pose.pose.position.y << "\tz: " << pose.pose.position.z << std::endl;
}

void markposeCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    std::cout << "估计位置x: " << msg->position.x / 1000 << "\ty: " << msg->position.y / 1000 
            << "\tz: " << msg->position.z /1000 << std::endl;
    std::cout << (fabs(-0.5 - (msg->position.x / 1000))) << " " << (fabs(-0.5 - (msg->position.y / 1000)));
    if ( (fabs(-0.5 - (msg->position.x / 1000)) < 0.2) && (fabs(-0.5 - (msg->position.y / 1000)) < 0.2) )
    {
        if (value_pose == true)
        {
            std::cout << "找到降落目标" << std::endl;
            find_mark = true;
            estimate_pose = *msg;
            value_pose = false;
            estimate_pose.position.x /= 1000.0;
            estimate_pose.position.y /= 1000.0;
            estimate_pose.position.z /= 1000.0;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_land");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber mark_pose_sub = nh.subscribe<geometry_msgs::Pose>("/quadrotor/pose", 10, &markposeCallback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int run_pose_num = 0;

    // estimate_pose.position.x = 0.293;
    // estimate_pose.position.y = 0.293;
    // estimate_pose.position.z = 2;

    while (ros::ok())
    {
        if (pose.pose.position.z <= 0.4)
        {
            break;
        }
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 无人机水平绕圆
        if (find_mark == false)
        {
            run_circle(pose);
        }
        // 找到降落目标
        else
        {
            if (run_pose_num < 100)
            {
                pose.pose.position.x += -1.0 * (estimate_pose.position.x + 0.2) * 0.01;
                pose.pose.position.y += -1.0 * (estimate_pose.position.y + 0.2) * 0.01;
                run_pose_num++;
            }
            if (run_pose_num >= 100)
            {
                pose.pose.position.z -= 0.01;
            }
            if (pose.pose.position.z < 0.3)
            {
                break;
            }
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled");
        last_request = ros::Time::now();
    }

    return 0;
}
