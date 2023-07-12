#include "ros/ros.h"
#include "tiago_iaslab_simulation/Objs.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment2/RobotAction.h>
#include <cstdlib>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<tiago_iaslab_simulation::Objs>("human_objects_srv");
    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;
    std::vector<int> id_sequence;

    if (client.call(srv))
    {
        // for (int i = 0; i < srv.response.ids.size(); i++)
        // {
        //     id_sequence.push_back(srv.response.ids[i]);
        // }
        // id_sequence = srv.response.ids;
        ROS_INFO("1st: %d", srv.response.ids[0]);
        ROS_INFO("2nd: %d", srv.response.ids[1]);
        ROS_INFO("3rd: %d", srv.response.ids[2]);
        ROS_INFO("done");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }


    actionlib::SimpleActionClient<assignment2::RobotAction> ac("robot", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");
    
    assignment2::RobotGoal goal;
    goal.pose = {8, -2, 0, 0, 0, -0.35, 0.94};
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