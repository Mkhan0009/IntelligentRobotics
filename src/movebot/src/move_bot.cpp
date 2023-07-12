#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_bot");

    if ( argc < 7 )
    {
        ROS_INFO(" ");
        ROS_INFO("\tUsage:");
        ROS_INFO(" ");
        ROS_INFO("\trosrun   x y z  r p y");
        ROS_INFO(" ");
        ROS_INFO("\twhere the list of arguments specify the target pose of /move_base in map");
        ROS_INFO(" ");
        return EXIT_FAILURE;
    }

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    
    while(ros::ok())
    {
        move_base_msgs::MoveBaseActionGoal msg;
        
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.goal_id.id = "goal B";
        msg.goal_id.stamp = ros::Time::now();
        msg.goal.target_pose.header.stamp = ros::Time::now();
        msg.goal.target_pose.header.frame_id = "map";
        msg.goal.target_pose.pose.position.x = atof(argv[1]);
        msg.goal.target_pose.pose.position.y = atof(argv[2]);
        msg.goal.target_pose.pose.position.z = atof(argv[3]);
        msg.goal.target_pose.pose.orientation.x = atof(argv[4]);
        msg.goal.target_pose.pose.orientation.y = atof(argv[5]);
        msg.goal.target_pose.pose.orientation.z = atof(argv[6]);
        msg.goal.target_pose.pose.orientation.w = atof(argv[7]);

        ROS_INFO("X: %f", msg.goal.target_pose.pose.position.x);
        ROS_INFO("Y: %f", msg.goal.target_pose.pose.position.y);
        pub.publish(msg);
        ros::spinOnce();
    }
    return 0;
}

/*
Setting goal: Frame:map,
Position(11,421, -0,392, 0,000),
Orientation(0,000, 0,000, 0,996, -0,085) = Angle: -2,971
*/