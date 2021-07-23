// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"

// System headers
//

// Library headers
//
#include <k4a/k4a.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

// Project headers
//

k4a_result_t K4AROSDeviceParams::GetDeviceConfig(k4a_device_configuration_t* configuration)
{
  configuration->depth_delay_off_color_usec = 0;
  configuration->disable_streaming_indicator = false;

  ROS_INFO_STREAM("Setting wired sync mode: " << wired_sync_mode);
  if (wired_sync_mode == 0)
  {
      configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
  }
  else if (wired_sync_mode == 1)
  {
      configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
  }
  else if (wired_sync_mode == 2)
  {
      configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
  }
  else
  {
      ROS_ERROR_STREAM("Invalid wired sync mode: " << wired_sync_mode);
      return K4A_RESULT_FAILED;
  }


  ROS_INFO_STREAM("Setting subordinate delay: " << subordinate_delay_off_master_usec);
  configuration->subordinate_delay_off_master_usec = subordinate_delay_off_master_usec;

  if (!color_enabled)
  {
    ROS_INFO_STREAM("Disabling RGB Camera");

    configuration->color_resolution = K4A_COLOR_RESOLUTION_OFF;
  }
  else
  {
    ROS_INFO_STREAM("Setting RGB Camera Format: " << color_format);

    if (color_format == "jpeg")
    {
      configuration->color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    }
    else if (color_format == "bgra")
    {
      configuration->color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    }
    else
    {
      ROS_ERROR_STREAM("Invalid RGB Camera Format: " << color_format);
      return K4A_RESULT_FAILED;
    }

    ROS_INFO_STREAM("Setting RGB Camera Resolution: " << color_resolution);

    if (color_resolution == "720P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_720P;
    }
    else if (color_resolution == "1080P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_1080P;
    }
    else if (color_resolution == "1440P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_1440P;
    }
    else if (color_resolution == "1536P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_1536P;
    }
    else if (color_resolution == "2160P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_2160P;
    }
    else if (color_resolution == "3072P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_3072P;
    }
    else
    {
      ROS_ERROR_STREAM("Invalid RGB Camera Resolution: " << color_resolution);
      return K4A_RESULT_FAILED;
    }
  }

  if (!depth_enabled)
  {
    ROS_INFO_STREAM("Disabling Depth Camera");

    configuration->depth_mode = K4A_DEPTH_MODE_OFF;
  }
  else
  {
    ROS_INFO_STREAM("Setting Depth Camera Mode: " << depth_mode);

    if (depth_mode == "NFOV_2X2BINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    }
    else if (depth_mode == "NFOV_UNBINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    }
    else if (depth_mode == "WFOV_2X2BINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    }
    else if (depth_mode == "WFOV_UNBINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    }
    else if (depth_mode == "PASSIVE_IR")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
    }
    else
    {
      ROS_ERROR_STREAM("Invalid Depth Camera Mode: " << depth_mode);
      return K4A_RESULT_FAILED;
    }
  }

  ROS_INFO_STREAM("Setting Camera FPS: " << fps);

  if (fps == 5)
  {
    configuration->camera_fps = K4A_FRAMES_PER_SECOND_5;
  }
  else if (fps == 15)
  {
    configuration->camera_fps = K4A_FRAMES_PER_SECOND_15;
  }
  else if (fps == 30)
  {
    configuration->camera_fps = K4A_FRAMES_PER_SECOND_30;
  }
  else
  {
    ROS_ERROR_STREAM("Invalid Camera FPS: " << fps);
    return K4A_RESULT_FAILED;
  }

  // Ensure that if RGB and depth cameras are enabled, we ask for synchronized frames
  if (depth_enabled && color_enabled)
  {
    configuration->synchronized_images_only = true;
  }
  else
  {
    configuration->synchronized_images_only = false;
  }

  // Ensure that the "point_cloud" option is not used with passive IR mode, since they are incompatible
  if (point_cloud && (configuration->depth_mode == K4A_DEPTH_MODE_PASSIVE_IR))
  {
    ROS_ERROR_STREAM("Incompatible options: cannot generate point cloud if depth camera is using PASSIVE_IR mode.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that point_cloud is enabled if using rgb_point_cloud
  if (rgb_point_cloud && !point_cloud)
  {
    ROS_ERROR_STREAM("Incompatible options: cannot generate RGB point cloud if point_cloud is not enabled.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that color camera is enabled when generating a color point cloud
  if (rgb_point_cloud && !color_enabled)
  {
    ROS_ERROR_STREAM("Incompatible options: cannot generate RGB point cloud if color camera is not enabled.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that color image contains RGB pixels instead of compressed JPEG data.
  if (rgb_point_cloud && color_format == "jpeg")
  {
    ROS_ERROR_STREAM("Incompatible options: cannot generate RGB point cloud if color format is JPEG.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that target IMU rate is feasible
  if (imu_rate_target == 0)
  {
    imu_rate_target = IMU_MAX_RATE;
    ROS_INFO_STREAM("Using default IMU rate. Setting to maximum: " << IMU_MAX_RATE << " Hz.");
  }

  if (imu_rate_target < 0 || imu_rate_target > IMU_MAX_RATE)
  {
    ROS_ERROR_STREAM("Incompatible options: desired IMU rate of " << imu_rate_target << "is not supported.");
    return K4A_RESULT_FAILED;
  }

  int div = IMU_MAX_RATE / imu_rate_target;
  float imu_rate_rounded = IMU_MAX_RATE / div;
  // Since we will throttle the IMU by averaging div samples together, this is the
  // achievable rate when rouded to the nearest whole number div.

  ROS_INFO_STREAM("Setting Target IMU rate to " << imu_rate_rounded << " (desired: " << imu_rate_target << ")");

  return K4A_RESULT_SUCCEEDED;
}

void K4AROSDeviceParams::Help()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)                                   \
  ROS_INFO("#param_variable - #param_type : param_help_string (#param_default_val)");

  ROS_PARAM_LIST
#undef LIST_ENTRY
}

void K4AROSDeviceParams::Print()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)                                   \
  ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

  ROS_PARAM_LIST
#undef LIST_ENTRY
}



void K4AROSDeviceParams::ReadFromFile(std::ifstream& fin, k4a::calibration& calibration)
{
    const YAML::Node doc = YAML::Load(fin);
    const YAML::Node depth_to_rgb = doc["extrinsics"]["depth_to_rgb"];
    ParseRows(depth_to_rgb, calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR]);
    const YAML::Node rgb_to_depth = doc["extrinsics"]["rgb_to_depth"];
    ParseRows(rgb_to_depth, calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR]);
    ParseRows(rgb_to_depth, calibration.color_camera_calibration.extrinsics);

    const YAML::Node rgb_intrinsics = doc["intrinsic_color"];
    ParseIntrinsicParameters(rgb_intrinsics, calibration.color_camera_calibration.intrinsics);
    const YAML::Node depth_intrinsics = doc["intrinsic_depth"];
    ParseIntrinsicParameters(depth_intrinsics, calibration.depth_camera_calibration.intrinsics);
}

void K4AROSDeviceParams::ParseRows(const YAML::Node& rows, k4a_calibration_extrinsics_t& calib)
{
  calib.rotation[0] = rows[0][0].as<float>();
  calib.rotation[1] = rows[0][1].as<float>();
  calib.rotation[2] = rows[0][2].as<float>();
  calib.rotation[3] = rows[1][0].as<float>();
  calib.rotation[4] = rows[1][1].as<float>();
  calib.rotation[5] = rows[1][2].as<float>();
  calib.rotation[6] = rows[2][0].as<float>();
  calib.rotation[7] = rows[2][1].as<float>();
  calib.rotation[8] = rows[2][2].as<float>();

  calib.translation[0] = rows[0][3].as<float>();
  calib.translation[1] = rows[1][3].as<float>();
  calib.translation[2] = rows[2][3].as<float>();
}

void K4AROSDeviceParams::ParseIntrinsicParameters(const YAML::Node& node, k4a_calibration_intrinsics_t& calib){
  calib.parameters.param.k1 = node["k1"].as<float>();
  calib.parameters.param.k2 = node["k2"].as<float>();
  calib.parameters.param.k3 = node["k3"].as<float>();
  calib.parameters.param.k4 = node["k4"].as<float>();
  calib.parameters.param.k5 = node["k5"].as<float>();
  calib.parameters.param.k6 = node["k6"].as<float>();
  calib.parameters.param.p1 = node["p1"].as<float>();
  calib.parameters.param.p2 = node["p2"].as<float>();

  calib.parameters.param.fx = node["camera_matrix"][0][0].as<float>();
  calib.parameters.param.fy = node["camera_matrix"][1][1].as<float>();
  calib.parameters.param.cx = node["camera_matrix"][0][2].as<float>();
  calib.parameters.param.cy = node["camera_matrix"][1][2].as<float>();

  calib.parameters.v[0] = calib.parameters.param.cx;
  calib.parameters.v[1] = calib.parameters.param.cy;
  calib.parameters.v[2] = calib.parameters.param.fx;
  calib.parameters.v[3] = calib.parameters.param.fy;
  calib.parameters.v[4] = calib.parameters.param.k1;
  calib.parameters.v[5] = calib.parameters.param.k2;
  calib.parameters.v[6] = calib.parameters.param.k3;
  calib.parameters.v[7] = calib.parameters.param.k4;
  calib.parameters.v[8] = calib.parameters.param.k5;
  calib.parameters.v[9] = calib.parameters.param.k6;
  calib.parameters.v[10] = calib.parameters.param.codx;
  calib.parameters.v[11] = calib.parameters.param.cody;
  calib.parameters.v[12] = calib.parameters.param.p2;
  calib.parameters.v[13] = calib.parameters.param.p1;
  calib.parameters.v[14] = calib.parameters.param.metric_radius;
}
