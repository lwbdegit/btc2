#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <unordered_map>

#define HASH_P 116101
#define MAX_N 10000000000

typedef struct ConfigSetting {
  /* for submap process*/
  double cloud_ds_size_ = 0.25;

  /* for binary descriptor*/
  int useful_corner_num_ = 30;
  float plane_merge_normal_thre_;
  float plane_merge_dis_thre_;
  float plane_detection_thre_ = 0.01;
  float voxel_size_ = 1.0;
  int voxel_init_num_ = 10;
  int proj_plane_num_ = 1;
  float proj_image_resolution_ = 0.5;
  float proj_image_high_inc_ = 0.5;
  float proj_dis_min_ = 0;
  float proj_dis_max_ = 5;
  float summary_min_thre_ = 10;
  int line_filter_enable_ = 0;

  /* for triangle descriptor */
  float descriptor_near_num_ = 10;
  float descriptor_min_len_ = 1;
  float descriptor_max_len_ = 10;
  float non_max_suppression_radius_ = 3.0;
  float std_side_resolution_ = 0.2;

  /* for place recognition*/
  int skip_near_num_ = 20;
  int candidate_num_ = 50;
  int sub_frame_num_ = 10;
  float rough_dis_threshold_ = 0.03;
  float similarity_threshold_ = 0.7;
  float icp_threshold_ = 0.5;
  float normal_threshold_ = 0.1;
  float dis_threshold_ = 0.3;

  /* extrinsic for lidar to vehicle*/
  Eigen::Matrix3d rot_lidar_to_vehicle_;
  Eigen::Vector3d t_lidar_to_vehicle_;

  /* for gt file style*/
  int gt_file_style_ = 0;

} ConfigSetting;

typedef struct BinaryDescriptor {
  std::vector<bool> occupy_array_;
  unsigned char summary_;
  Eigen::Vector3d location_;
} BinaryDescriptor;

// Binary Triangle Descriptor
typedef struct BTC {
  Eigen::Vector3d triangle_;
  Eigen::Vector3d angle_;
  Eigen::Vector3d center_;
  unsigned short frame_number_;
  BinaryDescriptor binary_A_;
  BinaryDescriptor binary_B_;
  BinaryDescriptor binary_C_;
  unsigned short match_frame_number_;
} BTC;

typedef struct Plane {
  pcl::PointXYZINormal p_center_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float d_ = 0;
  int id_ = 0;
  int sub_plane_num_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
} Plane;

typedef struct BTCMatchList {
  std::vector<std::pair<BTC, BTC>> match_list_;
  std::pair<int, int> match_id_;
  int match_frame_;
  double mean_dis_;
} BTCMatchList;

struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

class VOXEL_LOC {
 public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <>
struct hash<VOXEL_LOC> {
  int64 operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
}  // namespace std

class BTC_LOC {
 public:
  int64_t x, y, z, a, b, c;

  BTC_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int64_t va = 0,
          int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const BTC_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
    // return (x == other.x && y == other.y && z == other.z && a == other.a &&
    //         b == other.b && c == other.c);
  }
};

namespace std {
template <>
struct hash<BTC_LOC> {
  int64 operator()(const BTC_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
}  // namespace std

class OctoTree {
 public:
  ConfigSetting config_setting_;
  std::vector<Eigen::Vector3d> voxel_points_;
  std::shared_ptr<Plane> plane_ptr_;
  int layer_;
  int octo_state_;  // 0 is end of tree, 1 is not
  int merge_num_ = 0;
  bool is_project_ = false;
  std::vector<Eigen::Vector3d> project_normal;
  bool is_publish_ = false;
  OctoTree *leaves_[8];
  double voxel_center_[3];  // x, y, z
  float quater_length_;
  bool init_octo_;

  // for plot
  bool is_check_connect_[6];
  bool connect_[6];
  OctoTree *connect_tree_[6];

  OctoTree(const ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    voxel_points_.clear();
    octo_state_ = 0;
    layer_ = 0;
    init_octo_ = false;
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    // for plot
    for (int i = 0; i < 6; i++) {
      is_check_connect_[i] = false;
      connect_[i] = false;
      connect_tree_[i] = nullptr;
    }
    plane_ptr_.reset(new Plane);
  }
  void init_plane();
  void init_octo_tree();
};