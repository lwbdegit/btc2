/*
用于回环检测
*/
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/filesystem.hpp>
#include <signal.h>
#include <thread>

#include "include/btc.h"
#include "include/utils.h"

bool flg_exit_ = false;
void SigHandle(int sig) {
  flg_exit_ = true;
  std::cout << "recv signal" << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rs_btc_place_recognition");
  ros::NodeHandle nh;
  signal(SIGINT, SigHandle);
  
  std::string setting_path = "";
  std::string pcds_dir = "";
  std::string pose_file = "";
  std::string result_file = "";
  double cloud_overlap_thr = 0.5;
  bool calc_gt_enable = false;
  bool read_bin = true;
  bool enable_transform = false;
  int num_localmap_need;
  double cloud_down_voxel;
  nh.param<double>("cloud_overlap_thr", cloud_overlap_thr, 0.5);
  nh.param<std::string>("setting_path", setting_path, "");
  nh.param<std::string>("pcds_dir", pcds_dir, "");
  nh.param<std::string>("pose_file", pose_file, "");
  nh.param<bool>("read_bin", read_bin, true);
  nh.param<bool>("enable_transform", enable_transform, true);
  nh.param<int>("num_localmap_need", num_localmap_need, -1);
  nh.param<double>("cloud_down_voxel", cloud_down_voxel, 0.5);

  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  ros::Publisher pubCurrentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentBinary =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubAllBinary =
      nh.advertise<sensor_msgs::PointCloud2>("/all_key_points", 100);
  ros::Publisher pubCloudPlane =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_all_plane", 100);
  ros::Publisher pubProjPlane =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_proj_plane", 100);

  ros::Publisher pubCurrentPose =
      nh.advertise<nav_msgs::Odometry>("/current_pose", 10);
  ros::Publisher pubMatchedPose =
      nh.advertise<nav_msgs::Odometry>("/matched_pose", 10);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedBinary =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubLoopStatus =
      nh.advertise<visualization_msgs::MarkerArray>("/loop_status", 100);
  ros::Publisher pubBTC =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
  ros::Publisher pubBTCList =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line_list", 10);

  std_msgs::ColorRGBA color_tp;
  std_msgs::ColorRGBA color_fp;
  std_msgs::ColorRGBA color_path;
  double scale_tp = 1.0;
  double scale_fp = 2.0;
  double scale_path = 0.5;
  color_tp.a = 1.0;
  color_tp.r = 0.0 / 255.0;
  color_tp.g = 255.0 / 255.0;
  color_tp.b = 0.0 / 255.0;

  color_fp.a = 1.0;
  color_fp.r = 1.0;
  color_fp.g = 0.0;
  color_fp.b = 0.0;

  color_path.a = 0.8;
  color_path.r = 255.0 / 255.0;
  color_path.g = 255.0 / 255.0;
  color_path.b = 255.0 / 255.0;

  ros::Rate loop(50000);
  ros::Rate slow_loop(10); // delay for clear visualization
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  ConfigSetting config_setting;
  load_config_setting(setting_path, config_setting);

  // pose is only for visulization and gt overlap calculation
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
  std::vector<double> time_list;
  load_evo_pose_with_time(pose_file, pose_list, time_list);
  std::string print_msg = "Successfully load pose file:" + pose_file +
                          ". pose size:" + std::to_string(pose_list.size()) +
                          " time size:" + std::to_string(time_list.size());
  ROS_INFO_STREAM(print_msg.c_str());
  if(pose_list.empty() || time_list.empty()){
    ROS_ERROR_STREAM("pose_list or time_list is empty!");
    return -1;
  }

  BtcDescManager *btc_manager = new BtcDescManager(config_setting);
  btc_manager->print_debug_info_ = true;

  std::vector<double> descriptor_time;
  std::vector<double> querying_time;
  std::vector<double> update_time;
  int triggle_loop_num = 0;
  int true_loop_num = 0;
  int false_loop_num = 0;
  bool finish = false;

  while (ros::ok() && !finish) {
    for (size_t pose_id = 0; pose_id < pose_list.size(); ++pose_id) {

      LINFO << "\nprocess: " << pose_id << "/" << pose_list.size() << REND;
      // load pcd
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI> transform_cloud;
      cloud->reserve(1000000);
      transform_cloud.reserve(1000000);
      // Get pose information, only for gt overlap calculation
      Eigen::Vector3d translation = pose_list[pose_id].first;
      Eigen::Matrix3d rotation = pose_list[pose_id].second;
      {
        // Load point cloud from pcd file
        std::stringstream ss;
        ss << pcds_dir << "/" << std::setfill('0') << std::setw(6) << pose_id
           << ".pcd";
        std::string pcd_file = ss.str();

        auto t_load_start = std::chrono::high_resolution_clock::now();
        // pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud)
        pcl::PCDReader reader;
        if (reader.read(pcd_file, *cloud) == -1) {
          ROS_ERROR_STREAM("Couldn't read file " << pcd_file);
          continue;
        }
        auto t_load_end = std::chrono::high_resolution_clock::now();
        std::cout << "[Node][Time] load cloud: " << time_inc(t_load_end, t_load_start)
                  << "ms, " << cloud->size() << std::endl;
        transform_cloud = *cloud;

        for (size_t j = 0; j < transform_cloud.size(); j++) {
          Eigen::Vector3d pv = point2vec(transform_cloud.points[j]);
          pv = rotation * pv + translation;
          transform_cloud.points[j] = vec2point(pv);
        }
      }

      // maintain localmap
      static pcl::PointCloud<pcl::PointXYZI> localmap_cloud;
      static int localmap_id = 0;
      if (num_localmap_need) {
        static int scan_used_num = 0;
        if (scan_used_num < num_localmap_need) {
          if (scan_used_num == 0) {
            localmap_cloud.clear();
          }
          localmap_cloud += transform_cloud;
          scan_used_num++;
          continue;
        } else {
          localmap_cloud += transform_cloud;
          down_sampling_voxel(localmap_cloud, cloud_down_voxel);
          scan_used_num = 0;
          ++localmap_id;
          std::cout << "[localmap] localmap_cloud size: "
                    << localmap_cloud.size() << std::endl;
        }
      } else {
        localmap_cloud = transform_cloud;
        localmap_id = pose_id;
      }

      // ===============================================================================
      // step1. Descriptor Extraction
      std::cout << "[Node][Description] pose id:" << pose_id <<" localmap_id: " <<localmap_id << std::endl;
      auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
      std::vector<BTC> btcs_vec;
      btc_manager->GenerateBtcDescs(localmap_cloud.makeShared(), localmap_id,
                                    btcs_vec);
      auto t_descriptor_end = std::chrono::high_resolution_clock::now();
      descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));

      // step2. Searching Loop
      auto t_query_begin = std::chrono::high_resolution_clock::now();
      // search result
      std::pair<int, double> search_result(-1, 0); // <candidate_id, plane icp score>
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      loop_transform.first << 0, 0, 0;
      loop_transform.second = Eigen::Matrix3d::Identity();
      std::vector<std::pair<BTC, BTC>> loop_std_pair;
      
      if (pose_id > config_setting.skip_near_num_) {
        if (btcs_vec.size() == 0) {
          // getchar();
        }
        btc_manager->SearchLoop(btcs_vec, search_result, loop_transform,
                                loop_std_pair);
      }
      if (search_result.first > 0) {
        std::cout << "[Node][Loop Detection] triggle loop: " << localmap_id << "--"
                  << search_result.first << ", score:" << search_result.second
                  << std::endl;
      }
      auto t_query_end = std::chrono::high_resolution_clock::now();
      querying_time.push_back(time_inc(t_query_end, t_query_begin));

      // step3. Add descriptors to the database
      auto t_map_update_begin = std::chrono::high_resolution_clock::now();
      btc_manager->AddBtcDescs(btcs_vec);
      auto t_map_update_end = std::chrono::high_resolution_clock::now();
      update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
      std::cout << "[Node][Time] descriptor extraction: "
                << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
                << "query: " << time_inc(t_query_end, t_query_begin) << "ms, "
                << "update map: "
                << time_inc(t_map_update_end, t_map_update_begin) << "ms"
                << std::endl;

      // key_cloud_vec_ is used to calc overlap,down sample to save memory
      down_sampling_voxel(localmap_cloud, cloud_down_voxel * 1.);
      btc_manager->key_cloud_vec_.push_back(localmap_cloud.makeShared());
      // ===============================================================================

      //=== visulizaion
      publish_std_list(btcs_vec, pubBTCList);

      // publish current source cloud
      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(localmap_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCurrentCloud.publish(pub_cloud);

      // publish current frame whole plane
      pcl::PointCloud<pcl::PointXYZINormal> plane_normal_cloud;
      if (btc_manager->plane_cloud_vec_.size()) {
        int num = 5;
        double inter = 0.1;
        int intensity = 100;
        int cnt = 0;
        // for (auto &plane_cloud : btc_manager->plane_cloud_vec_) {
        for (auto &plane_pt : btc_manager->plane_cloud_vec_.back()->points) {
          // if(++cnt > 5) break;
          const auto &pt = plane_pt;
          for (int i = 0; i < num; i++) {
            pcl::PointXYZINormal tmp;
            tmp.x = pt.x + i * inter * pt.normal_x;
            tmp.y = pt.y + i * inter * pt.normal_y;
            tmp.z = pt.z + i * inter * pt.normal_z;
            tmp.intensity = intensity + i * 10;
            plane_normal_cloud.push_back(tmp);
          }
        }
      }
      pcl::toROSMsg(plane_normal_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCloudPlane.publish(pub_cloud);
      std::cout << "[Node]current plane size: "
                << btc_manager->plane_cloud_vec_.size() << std::endl;

      // publish hitorical plane

      // publish projection planes
      pcl::PointCloud<pcl::PointXYZINormal> proj_plane_normal_cloud;
      if (btc_manager->proj_plane_->size()) {
        int num = 5;
        double inter = 0.1;
        int cnt = 0;
        for (auto &proj_plane : *btc_manager->proj_plane_) {
          if (++cnt > config_setting.proj_plane_num_)
            break;
          num = std::ceil(proj_plane->radius_ * 10);
          num = std::max(num, 2);
          for (int i = 0; i < num; i++) {
            const auto &pt = proj_plane->center_;
            const auto &pt_normal = proj_plane->normal_;
            pcl::PointXYZINormal tmp;
            tmp.x = pt.x() + i * inter * pt_normal.x();
            tmp.y = pt.y() + i * inter * pt_normal.y();
            tmp.z = pt.z() + i * inter * pt_normal.z();
            tmp.intensity = cnt;
            tmp.curvature = proj_plane->id_;
            tmp.normal_x= proj_plane->points_size_;
            tmp.normal_y= proj_plane->sub_plane_num_;
            tmp.normal_z= proj_plane->radius_;
            proj_plane_normal_cloud.push_back(tmp);
          }
        }
      }
      pcl::toROSMsg(proj_plane_normal_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubProjPlane.publish(pub_cloud);
      std::cout << "[Node]proj plane size: " << btc_manager->proj_plane_->size()
                << std::endl;

      // publish current BTC descriptor points
      pcl::PointCloud<pcl::PointXYZI> key_points_cloud;
      for (auto var : btc_manager->history_binary_list_.back()) {
        pcl::PointXYZI pi;
        pi.x = var.location_[0];
        pi.y = var.location_[1];
        pi.z = var.location_[2];
        key_points_cloud.push_back(pi);
      }
      if (key_points_cloud.size()) {
        pcl::toROSMsg(key_points_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCurrentBinary.publish(pub_cloud);
      } else {
        ROS_ERROR_STREAM("key_points_cloud is empty!");
      }
      std::cout<<"[Node]currnet binary size: " << key_points_cloud.size();
      // publish all BTC descriptor points
      key_points_cloud.clear();
      for (auto binray : btc_manager->history_binary_list_) {
        for (auto var : binray) {
          pcl::PointXYZI pi;
          pi.x = var.location_[0];
          pi.y = var.location_[1];
          pi.z = var.location_[2];
          pi.intensity = var.summary_;
          key_points_cloud.push_back(pi);
        }
      }
      if (key_points_cloud.size()) {
        pcl::toROSMsg(key_points_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubAllBinary.publish(pub_cloud);
      }
      std::cout<<", all binary size: " << key_points_cloud.size()<<std::endl;

      // publish pair
      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.ns = "colored_path";
      marker.id = pose_id;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      if (search_result.first >= 0) {
        triggle_loop_num++;
        Eigen::Matrix4d transform1 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d transform2 = Eigen::Matrix4d::Identity();
        publish_std(loop_std_pair, transform1, transform2, pubBTC);
        slow_loop.sleep();

        // publish matched key points
        pcl::PointCloud<pcl::PointXYZI> match_key_points_cloud;
        for (auto var :
             btc_manager->history_binary_list_[search_result.first]) {
          pcl::PointXYZI pi;
          pi.x = var.location_[0];
          pi.y = var.location_[1];
          pi.z = var.location_[2];
          pi.intensity = var.summary_;
          match_key_points_cloud.push_back(pi);
        }
        pcl::toROSMsg(match_key_points_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedBinary.publish(pub_cloud);

        // true positive
        pcl::PointCloud<pcl::PointXYZI>::Ptr matcher_cloud =
            btc_manager->key_cloud_vec_[search_result.first];
        pcl::PointCloud<pcl::PointXYZRGB> matched_cloud_rgb;
        int matched_cloud_size = matcher_cloud->size();
        matched_cloud_rgb.resize(matched_cloud_size);
        for (size_t i = 0; i < matched_cloud_size; i++) {
          auto& pi = matched_cloud_rgb.points[i];
          pi.x = matcher_cloud->points[i].x;
          pi.y = matcher_cloud->points[i].y;
          pi.z = matcher_cloud->points[i].z;
          pi.r = pi.g = pi.b = 0;
        }

        // add cloud color
        double cloud_overlap =
            calc_overlap(localmap_cloud.makeShared(), matcher_cloud, 0.5);
        if (cloud_overlap >= cloud_overlap_thr) {
          true_loop_num++;
          for (size_t i = 0; i < matched_cloud_size; i++) {
            auto &pi = matched_cloud_rgb.points[i];
            pi.g = 255;
          }
          marker.scale.x = scale_tp;
          marker.color = color_tp;
        } else {
          for (size_t i = 0; i < matched_cloud_size; i++) {
            auto &pi = matched_cloud_rgb.points[i];
            pi.r = 255;
          }
          marker.scale.x = scale_fp;
          marker.color = color_fp;
        }
        // puslish matched cloud rgb
        pcl::toROSMsg(matched_cloud_rgb, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCloud.publish(pub_cloud);

        slow_loop.sleep();
      } else {
        //=== clear rviz
        // cloud
        pcl::PointCloud<pcl::PointXYZINormal> empty_cloud;
        pcl::toROSMsg(empty_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedBinary.publish(pub_cloud);
        pubMatchedCloud.publish(pub_cloud);
        // pubCloudPlane.publish(pub_cloud);
        // pubProjPlane.publish(pub_cloud);

        // btc
        visualization_msgs::MarkerArray ma_line;
        visualization_msgs::Marker m_line;
        m_line.type = visualization_msgs::Marker::LINE_LIST;
        m_line.action = visualization_msgs::Marker::ADD;
        m_line.ns = "lines";
        m_line.header.frame_id = "camera_init";
        m_line.id = 0;
        for (int j = 0; j < 100 * 6; j++) {
          m_line.color.a = 0.00;
          ma_line.markers.push_back(m_line);
          m_line.id++;
        }
        ma_line.markers.push_back(m_line);
        pubBTC.publish(ma_line);

        // 
        if (pose_id > 0) {
          marker.scale.x = scale_path;
          marker.color = color_path;
        }
      }
      // publish path marker
      geometry_msgs::Point point1;
      point1.x = pose_list[pose_id - 1].first[0];
      point1.y = pose_list[pose_id - 1].first[1];
      point1.z = pose_list[pose_id - 1].first[2];
      geometry_msgs::Point point2;
      point2.x = pose_list[pose_id].first[0];
      point2.y = pose_list[pose_id].first[1];
      point2.z = pose_list[pose_id].first[2];
      marker.points.push_back(point1);
      marker.points.push_back(point2);
      marker_array.markers.push_back(marker);
      pubLoopStatus.publish(marker_array);

      loop.sleep();
      if (flg_exit_)
        break;
    }
    finish = true;
  }

  // publish time cost
  double mean_descriptor_time =
      std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) * 1.0 /
      descriptor_time.size();
  double mean_query_time =
      std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
      querying_time.size();
  double mean_update_time =
      std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
      update_time.size();
  std::cout << "[Node]Total submap number:" << pose_list.size()
            << ", triggle loop number:" << triggle_loop_num
            << ", true loop number:" << true_loop_num
            << ", recall: " << double(true_loop_num) / triggle_loop_num << std::endl;
  std::cout << "[Node]Mean time for descriptor extraction: " << mean_descriptor_time
            << "ms, query: " << mean_query_time
            << "ms, update: " << mean_update_time << "ms, total: "
            << mean_descriptor_time + mean_query_time + mean_update_time << "ms"
            << std::endl;
  return 0;
}