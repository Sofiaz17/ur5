
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
bool debug_mode = false;

bool waitForServiceResponse(ros::ServiceClient &client, ur5::GetBrickPose &srv, int retries = 10, int delay_ms = 1000) {
    for (int i = 0; i < retries; ++i) {
        if (client.call(srv)) {
            return true;
        }
        ROS_WARN("%s Waiting for vision node to provide data...", info_name);
        ros::Duration(delay_ms / 1000.0).sleep();
    }
    return false;
}



/*!
    @brief The entry code of all the project. It calls everything else.
    @details It calls the following nodes: vision_node for bounding boxes, pointcloud_node to convert them into actual poses,
	desired_poses_node to retrive desired block poses, motion_planner to actually move the block.
    @param[in] int argc, char **argv: classical command line arguments.
    @return 0 if successful, 1 if something went wrong.
*/
int main(int argc, char **argv)
{
	/// Initialization

	ROS_INFO("%s Initialization phase", info_name);

	ros::init(argc, argv, "task_planner");
	ros::NodeHandle node;

	bool service_exit;
	// std::vector<ur5::TargetPose> desired_poses;
	// std::vector<ur5::TargetPose> actual_poses;

	std::vector<ur5::BlockParams> desired_poses;
	std::vector<ur5::BlockParams> actual_poses;


	// ros::ServiceClient vision_node = node.serviceClient<ur5::GetBrickPose>("get_brick_pose");
	ros::ServiceClient desired_poses_node = node.serviceClient<ur5::GetDesiredPoses>("get_desired_poses");
	ros::ServiceClient motion_planner = node.serviceClient<ur5::MoveBlock>("move_block");
	// vision_node.waitForExistence();
	desired_poses_node.waitForExistence();
	motion_planner.waitForExistence();

	// ur5::GetBrickPose get_brick_pose_service;
	ur5::GetDesiredPoses get_desired_poses_service;
	ur5::MoveBlock move_block_service;

	ros::param::get("/debug_mode", debug_mode);
	ROS_INFO("%s task_planner is ready!", info_name);

	ROS_INFO(" ");
	ROS_INFO("##############################");
	ROS_INFO("#  Here starts the workflow  #");
	ROS_INFO("##############################");
	ROS_INFO(" ");

	/// Vision Node

	if(debug_mode) ROS_INFO("%s Vision phase", info_name);
	ROS_INFO("%s Vision phase", info_name);

	//get_bounding_boxes_service.request is empty

	// service_exit = vision_node.call(get_bounding_boxes_service);
	// if(!service_exit){
	// 	ROS_ERROR("%s Failed to call service get_bounding_boxes", info_name);
	// 	return 1;
	// }
	//service_exit = vision_node.call(get_brick_pose_service);
	//  service_exit = waitForServiceResponse(vision_node, get_brick_pose_service);
    // if (!service_exit) {
    //     ROS_ERROR("%s Failed to call service get_brick_pose_service", info_name);
    //     return 1;
    // } else {
    //     ROS_INFO("get_brick_pose_service obtained!");
    // }

	
	// if(!service_exit){
	// 	ROS_ERROR("%s Failed to call service get_brick_pose_service", info_name);
	// 	return 1;
	// } else {
	// 	ROS_INFO("get_brick_pose_service obtained!");
	// }


	//actual_poses = get_brick_pose_service.response.poses; //pos blocchi visti da vision


//Roll pitch and yaw in Radians
float roll = 0, pitch = 0, yaw = 0.5236;    
Quaternionf qa;
qa = AngleAxisf(roll, Vector3f::UnitX())
    * AngleAxisf(pitch, Vector3f::UnitY())
    * AngleAxisf(yaw, Vector3f::UnitZ());
std::cout << "Quaternion" << std::endl << qa.coeffs() << std::endl;

	
  	// ur5::BlockParams block;
    // block.label = "X1-Y4-Z2";
    // block.pose.position.x = 0.4;
    // block.pose.position.y = 0.4;
    // block.pose.position.z = 0.87;
    // block.pose.orientation.x = q.x();
    // block.pose.orientation.y = q.y();
    // block.pose.orientation.z = q.z();
    // block.pose.orientation.w = q.w();

    // actual_poses.push_back(block);


	actual_poses.push_back(ur5::BlockParams());
	actual_poses.at(0).label = "X1-Y4-Z2";
	actual_poses.at(0).pose.position.x = 0.4;
	actual_poses.at(0).pose.position.y = 0.4;
	actual_poses.at(0).pose.position.z = 0.87;
	actual_poses.at(0).pose.orientation.x = qa.x();
	actual_poses.at(0).pose.orientation.y = qa.y();
	actual_poses.at(0).pose.orientation.z = qa.z();
	actual_poses.at(0).pose.orientation.w = qa.w();



  	// desired_poses.push_back(ur5::BlockParams());
	// desired_poses.at(0).label = "X1-Y4-Z2";
	// desired_poses.at(0).pose.position.x = -0.1;
	// desired_poses.at(0).pose.position.y = 0.35;
	// desired_poses.at(0).pose.position.z = 0.87;
	// desired_poses.at(0).pose.orientation.x = qa.x();
	// desired_poses.at(0).pose.orientation.y = qa.y();
	// desired_poses.at(0).pose.orientation.z = qa.z();
	// desired_poses.at(0).pose.orientation.w = qa.w();

	/// Desired Poses Node

	if(debug_mode) ROS_INFO("%s Retrieving desired poses", info_name);
	ROS_INFO("%s Retrieving desired poses", info_name);

	// get_desired_poses_service.request is empty

	service_exit = desired_poses_node.call(get_desired_poses_service);
	if(!service_exit){
		ROS_ERROR("%s Failed to call service get_desired_poses_service", info_name);
		return 1;
	} else {
		ROS_INFO("get_desired_poses_service obtained!");
	}

	desired_poses = get_desired_poses_service.response.poses;



	/// Motion Planner

	if(debug_mode) ROS_INFO("%s Motion planning phase", info_name);
	ROS_INFO("%s Motion planning phase", info_name);

	/**actual_poses.push_back<ur5::TargetPose());
	actual_poses.at(0).label = "X1-Y1-Z2";
	actual_poses.at(0).position.x = 0.6;
	actual_poses.at(0).position.y = 0.6;
	actual_poses.at(0).position.z = 0.87;
	actual_poses.at(0).euler.x = 0;
	actual_poses.at(0).euler.y = 0;
	actual_poses.at(0).euler.z = 0.78;
	/**
	actual_poses.push_back<ur5::TargetPose());
	actual_poses.push_back<ur5::TargetPose());
	actual_poses.push_back<ur5::TargetPose());
	actual_poses.at(0).label = "zzz";
	actual_poses.at(1).label = "bbb";
	actual_poses.at(2).label = "aaa";
	/**/

ur5::BlockParams actual_block;
	for(ur5::BlockParams desired_block : desired_poses){

		std::vector<ur5::BlockParams>::iterator it = std::find_if(actual_poses.begin(), actual_poses.end(),
			[desired_block](ur5::BlockParams i){return i.label == desired_block.label;}
		);

		if(it == actual_poses.end()){
			ROS_WARN("%s Block \"%s\" in desired_poses has not been detected, it will be ignored", info_name, desired_block.label.c_str());
		} else {
			actual_block = *it;

			move_block_service.request.start_pose = actual_block;
			move_block_service.request.end_pose = desired_block;

			service_exit = motion_planner.call(move_block_service);
			if(!service_exit){
				ROS_ERROR("%s Failed to call service move_block", info_name);
				return 1;
			} else {
				ROS_INFO("move_block_service obtained!");
			}

			if(!move_block_service.response.success){
				ROS_WARN("%s motion_planner failed to move block \"%s\"", info_name, desired_block.label.c_str());
			} 
			actual_poses.erase(it);
		}
	}

	for(ur5::BlockParams remaining_block : actual_poses){
		ROS_WARN("%s Detected block \"%s\" with no desired_poses entry, it will be ignored", info_name, remaining_block.label.c_str());
	}


	/// End

	ROS_INFO("%s task_planner is over!", info_name);
	ROS_INFO(" ");
	ROS_INFO("##############################");
	ROS_INFO("#   Here ends the workflow   #");
	ROS_INFO("##############################");
	ROS_INFO(" ");
	return 0;
}
