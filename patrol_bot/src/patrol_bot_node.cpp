#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>

using MoveBaseClient=actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
//умный указатель на объект - клиент
boost::shared_ptr<MoveBaseClient> moveBaseClientPtr;


void done_callback(const actionlib::SimpleClientGoalState& state,
                   const move_base_msgs::MoveBaseResultConstPtr& result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Target is reached");
    }
    else
    {
        ROS_ERROR("move_base has failed");
    }
}


void feedback_callback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM("feedback "<<
                    " robot pose x" << feedback->base_position.pose.position.x <<
                    " y = "<<feedback->base_position.pose.position.y);
}

void active_callback()
{
    ROS_INFO_STREAM("goal is started");
}


void clickPointCallback(const geometry_msgs::PointStamped& point)
{
    ROS_INFO_STREAM(" get point "<<point.point.x<<" "<<point.point.y);
    //задаем ориентацию в целевой точке
    double target_angle = M_PI/2;


    //формируем структуру цели для move_base Action
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position = point.point;
    //задаем кватернион, соответствующий ориентации
    goal.target_pose.pose.orientation.z = sin(target_angle/2);
    goal.target_pose.pose.orientation.w = cos(target_angle/2);
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    //отправляем цель
    moveBaseClientPtr->sendGoal(goal,
                                done_callback,
                                active_callback,
                                feedback_callback
                                );
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "control_node");


    ros::NodeHandle node("~");

    moveBaseClientPtr.reset(new MoveBaseClient("/move_base", false));

    while( !moveBaseClientPtr->isServerConnected())
    {
        ros::spinOnce();
    }

    ROS_INFO_STREAM("server move_base is connected");

    ros::Subscriber point_sub = node.subscribe("/clicked_point", 1, clickPointCallback);

    ros::spin();

    return 0;
}
