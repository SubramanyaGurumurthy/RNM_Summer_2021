// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller_sim.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointPositionExampleControllerSim::init(hardware_interface::PositionJointInterface *hw,
                                          ros::NodeHandle& node_handle) {

  position_joint_interface_ = hw;
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }
  command_sub_ = node_handle.subscribe<std_msgs::Float64MultiArray>(std::string("joint_command"), 1,
      &JointPositionExampleControllerSim::setCommandCallback, this);

  return true;
}


void JointPositionExampleControllerSim::starting(const ros::Time& /* time */) {
    for (size_t i = 0; i < 7; ++i) {
        initial_pose_[i] = position_joint_handles_[i].getPosition();
        command_.push_back(initial_pose_[i]);
        position_joint_handles_[i].setCommand(initial_pose_[i]);
    }
    elapsed_time_ = ros::Duration(0.0);
}


void JointPositionExampleControllerSim::update(const ros::Time&, const ros::Duration& period) {
    elapsed_time_ += period;
    for (size_t i = 0; i < position_joint_handles_.size(); i++) {
        position_joint_handles_[i].setCommand(command_[i]);
    }
}

bool JointPositionExampleControllerSim::validateJointSpeed(size_t joint_id, double commanded) {
    return std::abs(command_[joint_id] - commanded) * 1000 < max_joint_speed;
}


void JointPositionExampleControllerSim::setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg) {
    for( size_t i = 0; i < command_.size(); ++i) {
        if (!validateJointSpeed(i, msg->data[i])) {
            ROS_WARN(
                    "Large motion command detected at joint %lu (current: %f - commanded: %f)!\n"
                    "This would likely cause an emergency stop with the real hardware\n"
                    "You need to command to poses that are reachable within the 1ms timeframe", i,
                    position_joint_handles_[i].getCommand(), command_[i]);
        }
    }
    double diff = (ros::Time::now() - last_command_time_).toSec();
    // Check moving average of the last differences
    if( last_diff_ > 0) {
        diff = (diff + last_diff_ * 9) / 10;
    }
    last_diff_ = diff;
    if(diff < 0.0005) {
        ROS_WARN("Received commands too quickly (%f [s]). Real hardware only handles joint command every 1 ms",
                diff);
    }
    last_command_time_ = ros::Time::now();
    command_ = msg->data;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleControllerSim,
                       controller_interface::ControllerBase)





