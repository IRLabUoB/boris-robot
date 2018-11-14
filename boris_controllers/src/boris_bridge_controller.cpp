#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <boris_controllers/boris_bridge_controller.h>

namespace boris_controllers {

BorisBridgeController::BorisBridgeController() {}

BorisBridgeController::~BorisBridgeController() {}

bool BorisBridgeController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_des_.resize(kdl_chain_.getNrOfJoints());
 
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        tau_des_(i) = 0.0;;
        q_des_(i) = joint_set_point_handles_[i].getPosition();
    }

    ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

    sub_command_  = n.subscribe("command_torques",1,&BorisBridgeController::receive_torques,this);

    return true;


}

void BorisBridgeController::starting(const ros::Time& time)
{
    // Initializing stiffness, damping, ext_torque and set point values
    for (size_t i = 0; i < joint_handles_.size(); i++) {
        tau_des_(i) = 0.0;
        q_des_(i) = joint_handles_[i].getPosition();
    }


}

void BorisBridgeController::update(const ros::Time& time, const ros::Duration& period)
{

    //Compute control law. This controller sets all variables for the JointImpedance Interface from kuka
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        // we use tau for the control and set the other references to zero
        joint_handles_[i].setCommand(tau_des_(i));
        joint_stiffness_handles_[i].setCommand(0.0);
        joint_damping_handles_[i].setCommand(0.0);
        joint_set_point_handles_[i].setCommand(joint_handles_[i].getPosition());
    }

}

void BorisBridgeController::receive_torques(const sensor_msgs::JointState &msg)
{
    for(size_t i=0; i<joint_handles_.size(); i++)
        tau_des_(i) = msg.effort.at(i);
    ROS_INFO_STREAM(tau_des_.data);
}

} // namespace

PLUGINLIB_EXPORT_CLASS( boris_controllers::BorisBridgeController, controller_interface::ControllerBase)
