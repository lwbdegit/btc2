#ifndef ROS_DEP_H
#define ROS_DEP_H

#include <cv_bridge/cv_bridge.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "btc_struct.h"

inline void publish_binary(const std::vector<BinaryDescriptor> &binary_list,
                    const Eigen::Vector3d &text_color,
                    const std::string &text_ns,
                    const ros::Publisher &text_publisher) {
  visualization_msgs::MarkerArray text_array;
  visualization_msgs::Marker text;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.ns = text_ns;
  text.color.a = 0.8;  // Don't forget to set the alpha!
  text.scale.z = 0.08;
  text.pose.orientation.w = 1.0;
  text.header.frame_id = "camera_init";
  for (size_t i = 0; i < binary_list.size(); i++) {
    text.pose.position.x = binary_list[i].location_[0];
    text.pose.position.y = binary_list[i].location_[1];
    text.pose.position.z = binary_list[i].location_[2];
    std::ostringstream str;
    str << std::to_string((int)(binary_list[i].summary_));
    text.text = str.str();
    text.scale.x = 0.5;
    text.scale.y = 0.5;
    text.scale.z = 0.5;
    text.color.r = text_color[0];
    text.color.g = text_color[1];
    text.color.b = text_color[2];
    text.color.a = 1;
    text.id++;
    text_array.markers.push_back(text);
  }
  for (int i = 1; i < 100; i++) {
    text.color.a = 0;
    text.id++;
    text_array.markers.push_back(text);
  }
  text_publisher.publish(text_array);
  return;
}

inline void publish_std_list(const std::vector<BTC> &btc_list,
                      const ros::Publisher &std_publisher) {
  // publish descriptor
  visualization_msgs::MarkerArray ma_line;
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "std";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.5;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "camera_init";
  m_line.id = 0;
  m_line.points.clear();
  m_line.color.r = 0;
  m_line.color.g = 1;
  m_line.color.b = 0;
  m_line.color.a = 1;
  for (auto var : btc_list) {
    geometry_msgs::Point p;
    p.x = var.binary_A_.location_[0];
    p.y = var.binary_A_.location_[1];
    p.z = var.binary_A_.location_[2];
    m_line.points.push_back(p);
    p.x = var.binary_B_.location_[0];
    p.y = var.binary_B_.location_[1];
    p.z = var.binary_B_.location_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.binary_C_.location_[0];
    p.y = var.binary_C_.location_[1];
    p.z = var.binary_C_.location_[2];
    m_line.points.push_back(p);
    p.x = var.binary_B_.location_[0];
    p.y = var.binary_B_.location_[1];
    p.z = var.binary_B_.location_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.binary_C_.location_[0];
    p.y = var.binary_C_.location_[1];
    p.z = var.binary_C_.location_[2];
    m_line.points.push_back(p);
    p.x = var.binary_A_.location_[0];
    p.y = var.binary_A_.location_[1];
    p.z = var.binary_A_.location_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
  }
  for (int j = 0; j < 1000 * 3; j++) {
    m_line.color.a = 0.00;
    ma_line.markers.push_back(m_line);
    m_line.id++;
  }
  std_publisher.publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}

// 给每个三角形描述子三条边画线
inline void publish_std(const std::vector<std::pair<BTC, BTC>> &match_std_list,
                 const Eigen::Matrix4d &transform1,
                 const ros::Publisher &std_publisher, bool pub_source) {
  // publish descriptor
  // bool transform_enable = true;
  visualization_msgs::MarkerArray ma_line;
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "single_BTC_lines";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.25;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "camera_init";
  m_line.id = 0;
  const int max_pub_cnt = 100;
  int pub_cnt = 1;
  for (auto var : match_std_list) {
    if (pub_cnt > max_pub_cnt) {
      break;
    }
    pub_cnt++;
    m_line.color.a = 0.8;
    m_line.points.clear();
    
    // 用于求解位姿的BTC
    if (var.second.match_frame_number_ == var.first.frame_number_) {
      m_line.scale.x = 0.3;
      m_line.color.a = 1;
    } else {
      m_line.scale.x = 0.15;
      m_line.color.a = 0.5;
    }
    m_line.color.r = 0.0 / 255;
    m_line.color.g = 0.0 / 255;
    m_line.color.b = 255.0 / 255;
    geometry_msgs::Point p;
    Eigen::Vector3d t_p;
    t_p = var.first.binary_A_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.first.binary_B_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    t_p = var.first.binary_C_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    t_p = var.first.binary_B_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    t_p = var.first.binary_C_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    t_p = var.first.binary_A_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    // debug
    // std_publisher.publish(ma_line);
    // std::cout << "var first: " << var.first.triangle_.transpose()
    //           << " , var second: " << var.second.triangle_.transpose()
    //           << std::endl;
    // getchar();
  }
  for (int j = 0; j < max_pub_cnt * 3; j++) {
    m_line.color.a = 0.00;
    ma_line.markers.push_back(m_line);
    m_line.id++;
  }
  std_publisher.publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}

inline void publish_std_pair(const std::vector<std::pair<BTC, BTC>> &match_std_list,
                      const Eigen::Matrix4d &transform1,
                      const Eigen::Matrix4d &transform2,
                      const ros::Publisher &std_publisher,
                      const Eigen::Vector3d& rgb) {
  // publish descriptor
  // bool transform_enable = true;
  visualization_msgs::MarkerArray ma_line;
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "BTC_pair_lines";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.25;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "camera_init";
  m_line.id = 0;
  const int max_pub_cnt = 100;
  int pub_cnt = 1;
  for (auto var : match_std_list) {
    if (pub_cnt > max_pub_cnt) {
      break;
    }
    pub_cnt++;
    m_line.points.clear();
    // target triangle
    // 用于求解位姿的BTC
    if (var.second.match_frame_number_ == var.first.frame_number_) {
      m_line.scale.x = 0.3;
      m_line.color.a = 1;
    } else {
      m_line.scale.x = 0.15;
      m_line.color.a = 0.5;
    }
    m_line.color.r = rgb(0) / 255;
    m_line.color.g = rgb(1) / 255;
    m_line.color.b = rgb(2) / 255;
    geometry_msgs::Point p;
    Eigen::Vector3d t_p;
    t_p = var.second.binary_A_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.second.binary_B_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    t_p = var.second.binary_C_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.second.binary_B_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    t_p = var.second.binary_C_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.second.binary_A_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    // source triangle
    m_line.color.r = 1;
    m_line.color.g = 1;
    m_line.color.b = 1;
    t_p = var.first.binary_A_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.first.binary_B_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    t_p = var.first.binary_C_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    t_p = var.first.binary_B_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    t_p = var.first.binary_C_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    t_p = var.first.binary_A_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    // debug
    // std_publisher.publish(ma_line);
    // std::cout << "var first: " << var.first.triangle_.transpose()
    //           << " , var second: " << var.second.triangle_.transpose()
    //           << std::endl;
    // getchar();
  }
  for (int j = 0; j < max_pub_cnt * 6; j++) {
    m_line.color.a = 0.00;
    ma_line.markers.push_back(m_line);
    m_line.id++;
  }
  std_publisher.publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}

inline void CalcQuation(const Eigen::Vector3d &vec, const int axis,
                 geometry_msgs::Quaternion &q) {
  Eigen::Vector3d x_body = vec;
  Eigen::Vector3d y_body(1, 1, 0);
  if (x_body(2) != 0) {
    y_body(2) = -(y_body(0) * x_body(0) + y_body(1) * x_body(1)) / x_body(2);
  } else {
    if (x_body(1) != 0) {
      y_body(1) = -(y_body(0) * x_body(0)) / x_body(1);
    } else {
      y_body(0) = 0;
    }
  }
  y_body.normalize();
  Eigen::Vector3d z_body = x_body.cross(y_body);
  Eigen::Matrix3d rot;

  rot << x_body(0), x_body(1), x_body(2), y_body(0), y_body(1), y_body(2),
      z_body(0), z_body(1), z_body(2);
  Eigen::Matrix3d rotation = rot.transpose();
  if (axis == 2) {
    Eigen::Matrix3d rot_inc;
    rot_inc << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    rotation = rotation * rot_inc;
  }
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

inline void pubPlane(const ros::Publisher &plane_pub, const std::string plane_ns,
              const int plane_id, const pcl::PointXYZINormal normal_p,
              const float radius, const Eigen::Vector3d rgb) {
  visualization_msgs::Marker plane;
  plane.header.frame_id = "camera_init";
  plane.header.stamp = ros::Time();
  plane.ns = plane_ns;
  plane.id = plane_id;
  plane.type = visualization_msgs::Marker::CUBE;
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = normal_p.x;
  plane.pose.position.y = normal_p.y;
  plane.pose.position.z = normal_p.z;
  geometry_msgs::Quaternion q;
  Eigen::Vector3d normal_vec(normal_p.normal_x, normal_p.normal_y,
                             normal_p.normal_z);
  CalcQuation(normal_vec, 2, q);
  plane.pose.orientation = q;
  plane.scale.x = 3.0 * radius;
  plane.scale.y = 3.0 * radius;
  plane.scale.z = 0.1;
  plane.color.a = 0.8;  // 0.8
  plane.color.r = fabs(rgb(0));
  plane.color.g = fabs(rgb(1));
  plane.color.b = fabs(rgb(2));
  plane.lifetime = ros::Duration();
  plane_pub.publish(plane);
}

#endif  // ROS_DEP_H