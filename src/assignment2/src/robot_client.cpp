#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment2/RobotAction.h>


int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_assignment2");

    // make sure the user entered the correct number or commands
    if ( argc < 7 )
    {
        ROS_INFO(" ");
        ROS_INFO("\tUsage:");
        ROS_INFO(" ");
        ROS_INFO("\trosrun robot_client x y z  angle(x y z w)");
        ROS_INFO(" ");
        ROS_INFO("\texample: assignment2 robot_client 11.37 -0.34 0 0 0 -0.35 0.94");
        ROS_INFO(" ");
        ROS_INFO("\twhere the list of arguments specify the target pose of /move_base/goal expressed in /map");
        ROS_INFO(" ");
        return EXIT_FAILURE;
    }

    actionlib::SimpleActionClient<assignment2::RobotAction> ac("robot", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");

    // set the goal from the user input
    assignment2::RobotGoal goal;
    float x = atof(argv[1]);
    float y = atof(argv[2]);
    float z = atof(argv[3]);
    float angle_x = atof(argv[4]);
    float angle_y = atof(argv[5]);
    float angle_z = atof(argv[6]);
    float angle_w = atof(argv[7]);
    goal.pose = {x, y, z, angle_x, angle_y, angle_z, angle_w};
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(1000.0));
    

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }

    return 0;
}
