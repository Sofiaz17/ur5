#include "ros/ros.h"
#include "ros/master.h"

#include "ur5/MoveRobot.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

// Name identifier for logging purposes
char info_name[] = " [robot_controller]:";

// Number of joints in the robot
int JOINT_SIZE = 8;

// Maximum angular speed of the robot (radians per second)
double MAX_ANGULAR_SPEED = 3.1415926535 / 10; // PI/10 rad/s
// Rate of update for controlling the robot (in Hz)
double UPDATE_RATE = 1000.0; // 1 kHz
// Maximum incremental change in joint positions per update cycle
double MAX_INCREMENT; // This will be calculated based on MAX_ANGULAR_SPEED and UPDATE_RATE
ros::Publisher joint_group_pos_controller_publisher;

/*!
    @brief Handles the move_robot service of type ur5::MoveRobot.
    @details Moves the robot smoothly from its current joint state to the desired joint configuration.
    Interpolates intermediate positions and publishes them for smooth motion.
    @param[in] req The desired joint configuration.
    @param[out] res True if the movement was successful, false otherwise.
    @return True if the movement was successful, false otherwise.
*/
bool move_robot(ur5::MoveRobot::Request &req, ur5::MoveRobot::Response &res)
{
    ROS_INFO("%s Initiating robot movement...", info_name);
    std::vector<double> joints(JOINT_SIZE); // Current joint positions
    std::vector<double> target_joints = req.joints.data; // Target joint positions
    std::vector<double> increments(JOINT_SIZE); // Incremental changes in joint positions
    int total_steps = 0; // Total steps to reach the target position
    ros::Rate loop_rate(UPDATE_RATE); // Rate at which positions are updated

    // Retrieve current joint states from the robot
    sensor_msgs::JointState msg = *ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
    for (int i = 0; i < msg.name.size(); ++i) {
        // Map received joint positions to the corresponding joint names
        if (msg.name.at(i) == "shoulder_pan_joint") joints.at(0) = msg.position.at(i);
        else if (msg.name.at(i) == "shoulder_lift_joint") joints.at(1) = msg.position.at(i);
        else if (msg.name.at(i) == "elbow_joint") joints.at(2) = msg.position.at(i);
        else if (msg.name.at(i) == "wrist_1_joint") joints.at(3) = msg.position.at(i);
        else if (msg.name.at(i) == "wrist_2_joint") joints.at(4) = msg.position.at(i);
        else if (msg.name.at(i) == "wrist_3_joint") joints.at(5) = msg.position.at(i);
        else if (msg.name.at(i) == "hand_1_joint") joints.at(6) = msg.position.at(i);
        else if (msg.name.at(i) == "hand_2_joint") joints.at(7) = msg.position.at(i);
        else ROS_WARN("%s Received unexpected joint name from ur5_generic: %s", info_name, msg.name.at(i).c_str());
    }

    // Check if the received joint configuration matches the expected size
    if (req.joints.data.size() != JOINT_SIZE) {
        ROS_WARN("%s Robot controller received %ld joints, but expected %d. Ignoring this target.", info_name, req.joints.data.size(), JOINT_SIZE);
        ROS_INFO("%s ...target ignored", info_name);
        res.success = false;
        return true;
    }

    // Calculate the maximum distance to be covered by any joint
    for (int i = 0; i < JOINT_SIZE; ++i) {
        increments.at(i) = std::abs(target_joints.at(i) - joints.at(i));
    }
    double max_distance = *(std::max_element(increments.begin(), increments.end()));
    total_steps = std::ceil(max_distance / MAX_INCREMENT);

    // Calculate incremental changes for each joint to achieve smooth motion
    for (int i = 0; i < JOINT_SIZE; ++i) {
        increments.at(i) = (target_joints.at(i) - joints.at(i)) / total_steps;
    }

    // Perform incremental movements to reach the target position
    for (int dt = 0; dt < total_steps; ++dt) {
        for (int i = 0; i < JOINT_SIZE; ++i) {
            joints.at(i) += increments.at(i);
        }
        req.joints.data = joints;
        joint_group_pos_controller_publisher.publish(req.joints);
        loop_rate.sleep();
    }

    // Notify when the target position is reached
    ROS_INFO("%s ...target reached!", info_name);
    res.success = true;
    return true;
}

/*!
    @brief Main function for the robot_controller node.
    @details Waits for initialization of ur5_generic.py, then advertises the move_robot service.
    @param[in] argc Number of command line arguments.
    @param[in] argv Array of command line arguments.
    @return 0 if successful, 1 if an error occurred.
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle node;

    // Wait until ur5_generic.py is ready
    ros::Rate wait_for_ur5_generic(10.0);
    bool ur5_generic_is_ready = false;
    ros::master::V_TopicInfo topic_infos;
    ROS_INFO("%s Waiting for homing procedure...", info_name);

    do {
        wait_for_ur5_generic.sleep();
        ros::master::getTopics(topic_infos);
        for (auto i : topic_infos) {
            if (i.name == "/ur5_generic_is_ready") ur5_generic_is_ready = true;
        }
    } while (!ur5_generic_is_ready);
    ROS_INFO("%s ...homing procedure completed!", info_name);

    // Advertise the move_robot service and joint position publisher
    ros::ServiceServer service = node.advertiseService("move_robot", move_robot);
    joint_group_pos_controller_publisher = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);

    // Retrieve parameters from the ROS parameter server
    ros::param::get("/joint_size", JOINT_SIZE);
    ros::param::get("/max_angular_speed", MAX_ANGULAR_SPEED);
    ros::param::get("/update_rate", UPDATE_RATE);
    MAX_INCREMENT = MAX_ANGULAR_SPEED / UPDATE_RATE;

    // Notify when the robot controller is ready
    ROS_INFO("%s Robot controller is ready!", info_name);
    ros::spin();

    return 0;
}
