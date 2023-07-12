#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment1/RobotAction.h>
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "sensor_msgs/LaserScan.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include <vector>
#include <cmath>

class RobotAction
{
    protected:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
        ros::Subscriber sub_robot_moving_;
        actionlib::SimpleActionServer<assignment1::RobotAction> as_;
        std::string action_name_;
        assignment1::RobotFeedback feedback_;
        assignment1::RobotResult result_;
        const static int THRESHOLD = 1.5;
        const std::string GOAL_NAME = "goal_B";
        bool succeeded_;

    public:
        RobotAction(std::string name) : 
            as_(nh_, name, boost::bind(&RobotAction::executeCB, this, _1), false), 
                action_name_(name)
        {
            //register the preempt callback
            as_.registerPreemptCallback(boost::bind(&RobotAction::preemptCB, this));
            as_.start();
        }

        ~RobotAction(void) {}

        void preemptCB()
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
        }

        // blocking callback to ensure that the robot get to the desired pose before
        // running any other callback. the callback will wait for the movement to be complete
        void waitForRobotCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& robot_status)
        {
            feedback_.feedback = "robot movement completed";
            as_.publishFeedback(feedback_);
            ros::spinOnce();
        }


        // perform the tasks of moving the robot to the desired location
        // then running the laser scan
        void executeCB(const assignment1::RobotGoalConstPtr &goal)
        {
            // set sleep durations
            ros::Duration duration1(45);
            ros::Duration duration2(5);
            succeeded_ = true;
            
            // move the robot to the desired pose
            moveRobotCB(goal->pose[0], goal->pose[1], goal->pose[2], goal->pose[3],
                            goal->pose[4], goal->pose[5], goal->pose[6]);
            
            // the simulation runs slowly and thus needs time to wait for the robot
            duration1.sleep();
            
            feedback_.feedback = "starting scan";
            as_.publishFeedback(feedback_);
            // run the laser scan
            sub_ = nh_.subscribe("/scan_raw", 1, &RobotAction::scanCB, this);
            // make sure we running the laser scan and wait for results
            while (sub_.getNumPublishers() < 1) { }
            duration2.sleep();

            if (succeeded_)
            {
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
                feedback_.feedback = "scaning finished";
                as_.publishFeedback(feedback_);
            }
            else
            {
                ROS_INFO("%s: Aborted", action_name_.c_str());
                //set the action state to aborted
                as_.setAborted(result_);
            }
            ros::spinOnce();
        }

        // laser scan callback that populations rostopic robot/result
        void scanCB(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
        {
            // define variables
            float x, y, angle, mag, mag_prev;
            std::vector<float>x_detections;
            std::vector<float>y_detections;
            angle = laser_scan->angle_increment;
            
            // the scan callback call will get called as many times as the subscriber is
            // called but we only need to call it once to fill the result
            if (result_.obstacle_locations_x.size() < 1)
            {
                feedback_.feedback = "scaning";
                as_.publishFeedback(feedback_);
            }
            else
            {
                feedback_.feedback = "scaning finished";
                as_.publishFeedback(feedback_);
            }
            
            // set a flag so we only pick up the edges of the obstacles
            bool run = true;
            for (int i = 1; i < laser_scan->ranges.size(); i++)
            {
                mag_prev = laser_scan->ranges[i - 1];
                mag = laser_scan->ranges[i];
                // the edges of the obstacles will jump in magnitude
                if (std::abs(mag - mag_prev) >= THRESHOLD)
                {
                    if (run)
                    {
                        // convert to cartesian coordinates from polar coordinates
                        x = mag * cos(angle);
                        y = mag * sin(angle);
                        x_detections.push_back(x);
                        y_detections.push_back(y);
                    }
                    run = false;
                }
                else
                {
                    run = true;
                }
            }
            // populate the result
            if (result_.obstacle_locations_x.size() < 1)
            {
                for (int point = 1; point < x_detections.size();)
                {
                    result_.obstacle_locations_x.push_back(
                        (x_detections[point - 1] + x_detections[point]) / 2.0);
                    result_.obstacle_locations_y.push_back(
                        (y_detections[point - 1] + y_detections[point]) / 2.0);
                    point = point + 2;
                }
            }
            // we know in advance there are exactly 4 obstacles
            if (result_.obstacle_locations_x.size() != 4 ||
                result_.obstacle_locations_y.size() != 4)
            {
                succeeded_ = false;
            }
        }

        void moveRobotCB(float x, float y, float z,
                        float angle_x, float angle_y, float angle_z, float angle_w)
        {
            feedback_.feedback = "starting robot";
            as_.publishFeedback(feedback_);
            pub_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);            
            // wait for the publisher to actually get the subscriber
            // this allows us to avoid using while(ros::ok()) {...} and run only once
            while (pub_.getNumSubscribers() < 1) { }
            // populate the MoveBaseActionGoal message
            move_base_msgs::MoveBaseActionGoal msg;
            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();
            msg.goal_id.id = GOAL_NAME;
            msg.goal_id.stamp = ros::Time::now();
            msg.goal.target_pose.header.stamp = ros::Time::now();
            msg.goal.target_pose.header.frame_id = "map";
            msg.goal.target_pose.pose.position.x = x;
            msg.goal.target_pose.pose.position.y = y;
            msg.goal.target_pose.pose.position.z = z;
            msg.goal.target_pose.pose.orientation.x = angle_x;
            msg.goal.target_pose.pose.orientation.y = angle_y;
            msg.goal.target_pose.pose.orientation.z = angle_z;
            msg.goal.target_pose.pose.orientation.w = angle_w;

            pub_.publish(msg);
            ros::spinOnce();

            feedback_.feedback = "robot moving";
            as_.publishFeedback(feedback_);
            // call the blocking callback to ensure we wait for the robot to complete its movement
            sub_robot_moving_ = nh_.subscribe("/move_base/result", 1, &RobotAction::waitForRobotCB, this);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot");
    RobotAction robot("robot");
    ros::spin();
    return 0;
}