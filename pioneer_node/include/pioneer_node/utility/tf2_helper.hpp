#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/quaternion.hpp"

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}