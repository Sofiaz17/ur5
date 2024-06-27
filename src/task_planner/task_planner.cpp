#include "ros/ros.h"

#include "ur5/GetDesiredPoses.h"
#include "ur5/MoveBlock.h"
#include "ur5/GetBrickPose.h"

#include "ur5/BlockParams.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

#include <vector>

using namespace Eigen;

char info_name[] = " [  task_planner  ]:";

/*!
	@defgroup TASK_MANAGER TASK_MANAGER
    @addtogroup TASK_MANAGER
    @{
*/

// /**
//  * @brief Waits for a ROS service to become available, with retries and delays.
//  * @param[in] client The ROS service client.
//  * @param[in] srv The service object to call.
//  * @param[in] retries Number of retry attempts.
//  * @param[in] delay_ms Delay in milliseconds between retries.
//  * @return true if the service becomes available, false otherwise.
//  */
// bool waitForService(ros::ServiceClient &client, ur5::GetBrickPose &srv, int retries = 10, int delay_ms = 1000) {
//     for (int i = 0; i < retries; ++i) {
//         if (client.call(srv)) {
//             return true;
//         }
//         ROS_WARN("%s Waiting for vision node...", info_name);
//         ros::Duration(delay_ms / 1000.0).sleep();
//     }
//     return false;
// }


/*!
    @brief Main entry point for the task_planner node.
    @details Initializes ROS node, calls necessary services for vision, desired poses, and motion planning, and manages the workflow.
    @param[in] argc Number of command line arguments.
    @param[in] argv Array of command line arguments.
    @return 0 if successful, 1 if an error occurred.
*/
int main(int argc, char **argv)
{
    // Initialization
    ROS_INFO("%s Initialization phase", info_name);

    ros::init(argc, argv, "task_planner");
    ros::NodeHandle node;

    bool service_ended;
    std::vector<ur5::BlockParams> desired_poses;
    std::vector<ur5::BlockParams> actual_poses;

    ros::ServiceClient vision_node = node.serviceClient<ur5::GetBrickPose>("get_brick_pose");
    ros::ServiceClient desired_poses_node = node.serviceClient<ur5::GetDesiredPoses>("get_desired_poses");
    ros::ServiceClient motion_planner = node.serviceClient<ur5::MoveBlock>("move_block");
    vision_node.waitForExistence();
    desired_poses_node.waitForExistence();
    motion_planner.waitForExistence();

    ur5::GetBrickPose get_brick_pose_service;
    ur5::GetDesiredPoses get_desired_poses_service;
    ur5::MoveBlock move_block_service;

    ROS_INFO("%s task_planner is ready!", info_name);

    ROS_INFO("%s Starting the workflow", info_name);

    // Vision Node
    ROS_INFO("%s Vision phase", info_name);

    service_ended = vision_node.call(get_brick_pose_service);
    if (!service_ended) {
        ROS_WARN("%s Failed to call service get_brick_pose_service", info_name);
        return 1;
    } else {
        ROS_INFO("%s Vision service completed", info_name);
    }

    actual_poses = get_brick_pose_service.response.poses;

    // Desired Poses Node
    ROS_INFO("%s Retrieving desired poses", info_name);

    service_ended = desired_poses_node.call(get_desired_poses_service);
    if(!service_ended){
        ROS_WARN("%s Failed to call service get_desired_poses_service", info_name);
        return 1;
    } else {
        ROS_INFO("%s Desired poses retrieved", info_name);
    }

    desired_poses = get_desired_poses_service.response.poses;

    // Motion Planner
    ROS_INFO("%s Motion planning phase", info_name);

    ur5::BlockParams block;
    for(ur5::BlockParams desired_block : desired_poses){

        auto it = std::find_if(actual_poses.begin(), actual_poses.end(),
            [desired_block](ur5::BlockParams i){return i.label == desired_block.label;}
        );

        if(it == actual_poses.end()){
            ROS_WARN("%s Block \"%s\" in desired_poses has not been detected, ignoring", info_name, desired_block.label.c_str());
        } else {
            block = *it;

            move_block_service.request.start_pose = block;
            move_block_service.request.end_pose = desired_block;

            service_ended = motion_planner.call(move_block_service);
            if(!service_ended){
                ROS_WARN("%s Failed to call service move_block", info_name);
                return 1;
            } else {
                ROS_INFO("%s Block movement planned", info_name);
            }

            if(!move_block_service.response.success){
                ROS_WARN("%s Failed to move block \"%s\"", info_name, desired_block.label.c_str());
            } 
            actual_poses.erase(it);
        }
    }

    for(ur5::BlockParams remaining_block : actual_poses){
        ROS_WARN("%s Detected block \"%s\" with no desired_poses entry, ignoring", info_name, remaining_block.label.c_str());
    }

    // End
    ROS_INFO("%s task_planner completed", info_name);
    return 0;
}


/*! @} */
