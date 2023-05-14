#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>

using MoveBaseClient=actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
//умный указатель на объект - клиент
boost::shared_ptr<MoveBaseClient> moveBaseClientPtr;

geometry_msgs::PointStamped path[5];
int next_add = 0;
int target_point = 0;
bool got_targets = false;
bool target_ready = false;

void goNextPoint();

void done_callback(const actionlib::SimpleClientGoalState& state,
                   const move_base_msgs::MoveBaseResultConstPtr& result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Target is reached");
        target_ready = true;
    }
    else
    {
        ROS_ERROR("move_base has failed");
    }
}


void feedback_callback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    //ROS_INFO_STREAM("feedback "<<
                    //" robot pose x" << feedback->base_position.pose.position.x <<
                    //" y = "<<feedback->base_position.pose.position.y);
}

void active_callback()
{
    ROS_INFO_STREAM("goal is started");
}


void clickPointCallback(const geometry_msgs::PointStamped& point)
{
    ROS_INFO_STREAM(" get point "<<point.point.x<<" "<<point.point.y);
    //задаем ориентацию в целевой точке
    path[next_add%5] = point;
    next_add++;
    if(next_add>4)
    {
    	if(!got_targets)
    		goNextPoint();
    	got_targets = true;
    }	
}

void goNextPoint()
{
	ROS_INFO("Go next point");
    double target_angle = atan((path[(target_point+1)%5].point.y-path[(target_point)%5].point.y)/(path[(target_point+1)%5].point.x-path[(target_point)%5].point.x));
    ROS_INFO_STREAM("Angle " << target_angle);
	//double target_angle = M_PI/2;

    //формируем структуру цели для move_base Action
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position = path[(target_point)%5].point;
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
	target_point++;
}

void timerCallback(const ros::TimerEvent&)
{
	ROS_INFO_STREAM("Timer");
	if(target_ready)
	{
		target_ready = false;
		if(got_targets)
    			goNextPoint();
		
	}
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "control_node");

    ros::NodeHandle node("~");
    
    ros::Timer timer = node.createTimer(ros::Duration(0.1), timerCallback);

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