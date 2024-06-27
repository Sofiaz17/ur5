#include "ros/ros.h"
#include "ur5/MoveBlock.h"
#include "ur5/BlockParams.h"
#include "sensor_msgs/JointState.h"
#include "ur5/MoveRobot.h"
#include "std_msgs/Float64MultiArray.h"
#include "ur5/motion_planner.h"
#include "../motion_planner_ur5/motion_planner.h"
#include "../motion_planner_ur5/motion_planner.cpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#define Z_GRIP 0.15
#define Z_APPROACH 0.10
#define INTERMEDIATE_POINTS_NUMBER 6
#define GRIPPER_TYPE "soft_2"

char info_name[] = " [ motion_planner ]:";
int JOINT_SIZE = 8;
ros::ServiceClient robot_controller;

/*!
    @brief Retrieves the current joint configuration of the robot.
    @details This function subscribes to the /ur5/joint_states topic to get the current joint states of the robot.
    @return A Vector8d containing the current joint positions of the robot.
*/
Vector8d get_joint_state()
{
    Vector8d joints;
    sensor_msgs::JointState msg = *ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
    for(int i=0; i<msg.name.size(); ++i){
        if(msg.name.at(i) == "shoulder_pan_joint") joints(0) = msg.position.at(i);
        else if(msg.name.at(i) == "shoulder_lift_joint") joints(1) = msg.position.at(i);
        else if(msg.name.at(i) == "elbow_joint") joints(2) = msg.position.at(i);
        else if(msg.name.at(i) == "wrist_1_joint") joints(3) = msg.position.at(i);
        else if(msg.name.at(i) == "wrist_2_joint") joints(4) = msg.position.at(i);
        else if(msg.name.at(i) == "wrist_3_joint") joints(5) = msg.position.at(i);
        else if(msg.name.at(i) == "hand_1_joint") joints(6) = msg.position.at(i);
        else if(msg.name.at(i) == "hand_2_joint") joints(7) = msg.position.at(i);
    }
    return joints;
}

/*!
    @brief Adjusts the gripper joints to achieve the desired width.
    @param[in] width The desired width of the gripper.
    @param[in] gripper_type The type of gripper being used.
    @param[out] desired_joints A reference to the vector containing the adjusted joint values.
*/
void set_gripper_joints(std::vector<double> &desired_joints, double width, std::string gripper_type)
{
    double opening_angle = atan2(0.5 * (width - 0.04), 0.06);
    desired_joints.at(JOINT_SIZE - 1) = opening_angle;
    desired_joints.at(JOINT_SIZE - 2) = opening_angle;
}


/*!
    @brief Service handler for moving a block using the robot.
    @details Calculates the necessary via points and uses them to plan a path to grip the block. The joint configurations required to follow this path are sent to the robot_controller node.
    @param[in] req The request containing the block's current and desired positions.
    @param[out] res The response indicating success or failure of the operation.
    @return True if the operation was successful, false otherwise.
*/
bool move_block(ur5::MoveBlock::Request &req, ur5::MoveBlock::Response &res)
{
    bool service_exit;
    ur5::MoveRobot move_robot_service;

    Vector8d actual_joints;
    Quaterniond fix_block_orientation;
    std::vector<double> desired_joints(JOINT_SIZE);

    Path joint_path;
    std::vector<Vector3d> intermediate_pos(INTERMEDIATE_POINTS_NUMBER);
    std::vector<Quaterniond> intermediate_quat(INTERMEDIATE_POINTS_NUMBER);

    ROS_INFO("%s Moving block \"%s\"...", info_name, req.start_pose.label.c_str());

    // set_intermediate_points(ur5::MoveBlock::Request &req,     Quaterniond fix_block_orientation;)

    // Move the block
    fix_block_orientation.x() = 0.7071;
    fix_block_orientation.y() = -0.7071;
    fix_block_orientation.z() = 0.0;
    fix_block_orientation.w() = 0.0;

    intermediate_pos.at(1) << req.start_pose.pose.position.x, req.start_pose.pose.position.y, req.start_pose.pose.position.z + Z_GRIP;
    intermediate_quat.at(1).x() = req.start_pose.pose.orientation.x;
    intermediate_quat.at(1).y() = req.start_pose.pose.orientation.y;
    intermediate_quat.at(1).z() = req.start_pose.pose.orientation.z;
    intermediate_quat.at(1).w() = req.start_pose.pose.orientation.w;
    intermediate_quat.at(1) = intermediate_quat.at(1) * fix_block_orientation;

    intermediate_pos.at(0) = intermediate_pos.at(1) + Vector3d(0.0, 0.0, Z_APPROACH);
    intermediate_quat.at(0) = intermediate_quat.at(1);

    intermediate_pos.at(2) = intermediate_pos.at(0);
    intermediate_quat.at(2) = intermediate_quat.at(0);

    intermediate_pos.at(4) << req.end_pose.pose.position.x, req.end_pose.pose.position.y, req.end_pose.pose.position.z + Z_GRIP;
    intermediate_quat.at(4).x() = req.end_pose.pose.orientation.x;
    intermediate_quat.at(4).y() = req.end_pose.pose.orientation.y;
    intermediate_quat.at(4).z() = req.end_pose.pose.orientation.z;
    intermediate_quat.at(4).w() = req.end_pose.pose.orientation.w;
    intermediate_quat.at(4) = intermediate_quat.at(4) * fix_block_orientation;

    intermediate_pos.at(3) = intermediate_pos.at(4) + Vector3d(0.0, 0.0, Z_APPROACH);
    intermediate_quat.at(3) = intermediate_quat.at(4);

    intermediate_pos.at(5) = intermediate_pos.at(3);
    intermediate_quat.at(5) = intermediate_quat.at(3);

    for (int i=0; i < INTERMEDIATE_POINTS_NUMBER; ++i){
        actual_joints = get_joint_state();
        joint_path = diffInverseKinQuaternions(actual_joints, intermediate_pos.at(i), intermediate_quat.at(i));

        int last_joint = joint_path.rows() - 1;
        desired_joints.at(0) = joint_path(last_joint, 0);
        desired_joints.at(1) = joint_path(last_joint, 1);
        desired_joints.at(2) = joint_path(last_joint, 2);
        desired_joints.at(3) = joint_path(last_joint, 3);
        desired_joints.at(4) = joint_path(last_joint, 4);
        desired_joints.at(5) = joint_path(last_joint, 5);
        desired_joints.at(6) = joint_path(last_joint, 6);
        desired_joints.at(7) = joint_path(last_joint, 7);

        /* Send joints to robot */
        move_robot_service.request.joints.data = desired_joints;
        service_exit = robot_controller.call(move_robot_service);
        if(!service_exit){
            ROS_ERROR("%s Failed to call service move_robot", info_name);
            return false;
        }
        if(!move_robot_service.response.success){
            ROS_WARN("%s robot_controller failed to reach the target", info_name);
            ROS_INFO("%s ...block \"%s\" ignored", info_name, req.start_pose.label.c_str());
            res.success = false;
            return true;
        }

        bool gripper_change = false;
        if (i == 1) {
            if (req.start_pose.label == "X2-Y2-Z2" || req.start_pose.label == "X2-Y2-Z2-FILLET") {
                set_gripper_joints(desired_joints, 0.15, GRIPPER_TYPE);
            } else {
                set_gripper_joints(desired_joints, 0.02, GRIPPER_TYPE);
            }
            gripper_change = true;
        } else if (i == 4) {
            set_gripper_joints(desired_joints, 0.35, GRIPPER_TYPE);
            gripper_change = true;
        }

        if (gripper_change) {
            /* Send joints to robot */
            move_robot_service.request.joints.data = desired_joints;
            service_exit = robot_controller.call(move_robot_service);
            if(!service_exit){
                ROS_ERROR("%s Failed to call service move_robot", info_name);
                return false;
            }
            if(!move_robot_service.response.success){
                ROS_WARN("%s robot_controller failed to reach the target", info_name);
                ROS_INFO("%s ...block \"%s\" ignored", info_name, req.start_pose.label.c_str());
                res.success = false;
                return true;
            }
        }
    }

    ROS_INFO("%s ...block \"%s\" moved!", info_name, req.start_pose.label.c_str());
    res.success = true;
    return true;
}

/*!
    @brief Main function for the motion planner.
    @details Initializes the node, moves the robot to the home position, and advertises the move_block service.
    @param[in] argc Number of command line arguments.
    @param[in] argv Array of command line arguments.
    @return 0 if the operation was successful, 1 otherwise.
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle node;

    ROS_INFO("%s Waiting for robot_controller", info_name);
    robot_controller = node.serviceClient<ur5::MoveRobot>("move_robot");
    robot_controller.waitForExistence();

    ROS_INFO("%s moving robot to new homing", info_name);
    ur5::MoveRobot move_robot_homing;
    bool service_exit;
    std::vector<double> desired_joints{-0.32, -0.58, -2.76, -1.63, -3.14, 3.49, 0.0, 0.0};
    set_gripper_joints(desired_joints, 0.3, GRIPPER_TYPE);

    /* Send joints to robot */
    move_robot_homing.request.joints.data = desired_joints;
    service_exit = robot_controller.call(move_robot_homing);
    if(!service_exit){
        ROS_ERROR("%s Failed to call service move_robot", info_name);
        return false;
    }
    if(!move_robot_homing.response.success){
        ROS_WARN("%s robot_controller failed to reach the new homing...", info_name);
    }

    ros::ServiceServer service = node.advertiseService("move_block", move_block);
    ros::param::get("/joint_size", JOINT_SIZE);

    ROS_INFO("%s motion_planner is ready!", info_name);
    ros::spin();

    return 0;
}
