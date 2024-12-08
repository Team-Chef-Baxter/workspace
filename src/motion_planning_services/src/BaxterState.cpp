#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char *argv[]) {
    
    ros::init(argc, argv, "robot_state_node");
    ros::NodeHandle n;
    ros::Publisher right_eef_pub = 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Forward Kinematics

    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d& eef_state = kinematic_state->getGlobalLinkTransform("right_gripper_base");


}