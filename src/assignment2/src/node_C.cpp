#include "ros/ros.h"

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cstdlib>
#include <vector>


void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL closed_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    // BEGIN_SUB_TUTORIAL pick1
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
    // of the cube). |br|
    // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
    // extra padding)
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
    // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
    // BEGIN_SUB_TUTORIAL place
    // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    // location in
    // verbose mode." This is a known issue and we are working on fixing it. |br|
    // Create a vector of placings to be attempted, currently only creating single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* While placing it is the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.5;
    place_location[0].place_pose.pose.position.z = 0.5;

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    group.place("object", place_location);
    // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 8 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(11);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table0";
    collision_objects[0].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table1
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table1";
    collision_objects[1].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[2].id = "table2";
    collision_objects[2].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.4;
    collision_objects[2].primitives[0].dimensions[1] = 0.2;
    collision_objects[2].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0;
    collision_objects[2].primitive_poses[0].position.y = 0.5;
    collision_objects[2].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[2].ADD;

    // BEGIN_SUB_TUTORIAL table3
    // Add the second table where we will be placing the cube.
    collision_objects[3].id = "table2";
    collision_objects[3].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.4;
    collision_objects[3].primitives[0].dimensions[1] = 0.2;
    collision_objects[3].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0;
    collision_objects[3].primitive_poses[0].position.y = 0.5;
    collision_objects[3].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[3].operation = collision_objects[3].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the objects that we will be manipulating
    collision_objects[4].header.frame_id = "panda_link0";
    collision_objects[4].id = "blue_hexagon";

    /* Define the primitive and its dimensions. */
    collision_objects[4].primitives.resize(1);
    collision_objects[4].primitives[0].type = collision_objects[4].primitives[0].BOX;
    collision_objects[4].primitives[0].dimensions.resize(3);
    collision_objects[4].primitives[0].dimensions[0] = 0.02;
    collision_objects[4].primitives[0].dimensions[1] = 0.02;
    collision_objects[4].primitives[0].dimensions[2] = 0.02;

    /* Define the pose of the object. */
    collision_objects[4].primitive_poses.resize(1);
    collision_objects[4].primitive_poses[0].position.x = 0.5;
    collision_objects[4].primitive_poses[0].position.y = 0;
    collision_objects[4].primitive_poses[0].position.z = 0.5;

    collision_objects[4].operation = collision_objects[4].ADD;

    // Define the objects that we will be manipulating
    collision_objects[5].header.frame_id = "panda_link0";
    collision_objects[5].id = "green_triangle";

    /* Define the primitive and its dimensions. */
    collision_objects[5].primitives.resize(1);
    collision_objects[5].primitives[0].type = collision_objects[5].primitives[0].BOX;
    collision_objects[5].primitives[0].dimensions.resize(3);
    collision_objects[5].primitives[0].dimensions[0] = 0.02;
    collision_objects[5].primitives[0].dimensions[1] = 0.02;
    collision_objects[5].primitives[0].dimensions[2] = 0.02;

    /* Define the pose of the object. */
    collision_objects[5].primitive_poses.resize(1);
    collision_objects[5].primitive_poses[0].position.x = 0.5;
    collision_objects[5].primitive_poses[0].position.y = 0;
    collision_objects[5].primitive_poses[0].position.z = 0.5;

    collision_objects[5].operation = collision_objects[5].ADD

    // Define the objects that we will be manipulating
    collision_objects[6].header.frame_id = "panda_link0";
    collision_objects[6].id = "red_cube";

    /* Define the primitive and its dimensions. */
    collision_objects[6].primitives.resize(1);
    collision_objects[6].primitives[0].type = collision_objects[6].primitives[0].BOX;
    collision_objects[6].primitives[0].dimensions.resize(3);
    collision_objects[6].primitives[0].dimensions[0] = 0.02;
    collision_objects[6].primitives[0].dimensions[1] = 0.02;
    collision_objects[6].primitives[0].dimensions[2] = 0.02;

    /* Define the pose of the object. */
    collision_objects[6].primitive_poses.resize(1);
    collision_objects[6].primitive_poses[0].position.x = 0.5;
    collision_objects[6].primitive_poses[0].position.y = 0;
    collision_objects[6].primitive_poses[0].position.z = 0.5;

    collision_objects[6].operation = collision_objects[6].ADD

    // Define the obstacles that we will need to avoid
    collision_objects[7].header.frame_id = "panda_link0";
    collision_objects[7].id = "gold_obs_0";

    /* Define the primitive and its dimensions. */
    collision_objects[7].primitives.resize(1);
    collision_objects[7].primitives[0].type = collision_objects[7].primitives[0].BOX;
    collision_objects[7].primitives[0].dimensions.resize(3);
    collision_objects[7].primitives[0].dimensions[0] = 0.02;
    collision_objects[7].primitives[0].dimensions[1] = 0.02;
    collision_objects[7].primitives[0].dimensions[2] = 0.02;

    /* Define the pose of the object. */
    collision_objects[7].primitive_poses.resize(1);
    collision_objects[7].primitive_poses[0].position.x = 0.5;
    collision_objects[7].primitive_poses[0].position.y = 0;
    collision_objects[7].primitive_poses[0].position.z = 0.5;

    collision_objects[7].operation = collision_objects[7].ADD


    // Define the obstacles that we will need to avoid
    collision_objects[8].header.frame_id = "panda_link0";
    collision_objects[8].id = "gold_obs_1";

    /* Define the primitive and its dimensions. */
    collision_objects[8].primitives.resize(1);
    collision_objects[8].primitives[0].type = collision_objects[8].primitives[0].BOX;
    collision_objects[8].primitives[0].dimensions.resize(3);
    collision_objects[8].primitives[0].dimensions[0] = 0.02;
    collision_objects[8].primitives[0].dimensions[1] = 0.02;
    collision_objects[8].primitives[0].dimensions[2] = 0.02;

    /* Define the pose of the object. */
    collision_objects[8].primitive_poses.resize(1);
    collision_objects[8].primitive_poses[0].position.x = 0.5;
    collision_objects[8].primitive_poses[0].position.y = 0;
    collision_objects[8].primitive_poses[0].position.z = 0.5;

    collision_objects[8].operation = collision_objects[8].ADD


    // Define the obstacles that we will need to avoid
    collision_objects[9].header.frame_id = "panda_link0";
    collision_objects[9].id = "gold_obs_2";

    /* Define the primitive and its dimensions. */
    collision_objects[9].primitives.resize(1);
    collision_objects[9].primitives[0].type = collision_objects[9].primitives[0].BOX;
    collision_objects[9].primitives[0].dimensions.resize(3);
    collision_objects[9].primitives[0].dimensions[0] = 0.02;
    collision_objects[9].primitives[0].dimensions[1] = 0.02;
    collision_objects[9].primitives[0].dimensions[2] = 0.02;

    /* Define the pose of the object. */
    collision_objects[9].primitive_poses.resize(1);
    collision_objects[9].primitive_poses[0].position.x = 0.5;
    collision_objects[9].primitive_poses[0].position.y = 0;
    collision_objects[9].primitive_poses[0].position.z = 0.5;

    collision_objects[9].operation = collision_objects[9].ADD

    // Define the obstacles that we will need to avoid
    collision_objects[10].header.frame_id = "panda_link0";
    collision_objects[10].id = "gold_obs_3";

    /* Define the primitive and its dimensions. */
    collision_objects[10].primitives.resize(1);
    collision_objects[10].primitives[0].type = collision_objects[10].primitives[0].BOX;
    collision_objects[10].primitives[0].dimensions.resize(3);
    collision_objects[10].primitives[0].dimensions[0] = 0.02;
    collision_objects[10].primitives[0].dimensions[1] = 0.02;
    collision_objects[10].primitives[0].dimensions[2] = 0.02;

    /* Define the pose of the object. */
    collision_objects[10].primitive_poses.resize(1);
    collision_objects[10].primitive_poses[0].position.x = 0.5;
    collision_objects[10].primitive_poses[0].position.y = 0;
    collision_objects[10].primitive_poses[0].position.z = 0.5;

    collision_objects[10].operation = collision_objects[10].ADD


    planning_scene_interface.applyCollisionObjects(collision_objects);
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(45.0);

    addCollisionObjects(planning_scene_interface);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    pick(group);

    ros::WallDuration(1.0).sleep();

    // place(group);

    ros::waitForShutdown();
    return 0;
}