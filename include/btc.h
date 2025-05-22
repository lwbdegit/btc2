#ifndef BTC_H
#define BTC_H
#include <ceres/ceres.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <execution>
#include <fstream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include "btc_struct.h"

// debug
#include "utils.h"
// RS debug
#define RSRESET "\033[0m"
#define RSBOLDRED "\033[1m\033[31m"     /* Bold Red */
#define RSBOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define RSBOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define RSBOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define RSBOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define LINFO (std::cout << RSBOLDGREEN)
#define LWARNING (std::cout << RSBOLDYELLOW)
#define LERROR (std::cout << RSBOLDRED)
#define LDEBUG (std::cout << RSBOLDCYAN)
#define LTITLE (std::cout << RSBOLDMAGENTA)
#define END (std::endl)
#define REND "\033[0m" << std::endl
#define PRINT printf

#define SOLVE_ONLY_ONE_PAIR false // 用1个BTC求解
#define ENABLE_PARALELL_FOR_LOOP false // 开启并行for循环

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size);

void load_config_setting(std::string &config_file,
                         ConfigSetting &config_setting);

double binary_similarity(const BinaryDescriptor &b1,
                         const BinaryDescriptor &b2);

bool binary_greater_sort(BinaryDescriptor a, BinaryDescriptor b);
bool plane_greater_sort(std::shared_ptr<Plane> plane1,
                        std::shared_ptr<Plane> plane2);

double calc_triangle_dis(
    const std::vector<std::pair<BTC, BTC>> &match_std_list);

double calc_binary_similaity(
    const std::vector<std::pair<BTC, BTC>> &match_std_list);

struct PlaneSolver {
  PlaneSolver(Eigen::Vector3d curr_point_, Eigen::Vector3d curr_normal_,
              Eigen::Vector3d target_point_, Eigen::Vector3d target_normal_)
      : curr_point(curr_point_),
        curr_normal(curr_normal_),
        target_point(target_point_),
        target_normal(target_normal_) {};
  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
                              T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;
    Eigen::Matrix<T, 3, 1> point_target(
        T(target_point.x()), T(target_point.y()), T(target_point.z()));
    Eigen::Matrix<T, 3, 1> norm(T(target_normal.x()), T(target_normal.y()),
                                T(target_normal.z()));
    residual[0] = norm.dot(point_w - point_target);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d curr_normal_,
                                     Eigen::Vector3d target_point_,
                                     Eigen::Vector3d target_normal_) {
    return (
        new ceres::AutoDiffCostFunction<PlaneSolver, 1, 4, 3>(new PlaneSolver(
            curr_point_, curr_normal_, target_point_, target_normal_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d curr_normal;
  Eigen::Vector3d target_point;
  Eigen::Vector3d target_normal;
};

class SolverResult {
public:
  SolverResult() { ; };
  std::vector<Eigen::Vector3d> src_vec_;
  std::vector<Eigen::Vector3d> ref_vec_;
  std::vector<std::pair<BTC, BTC>> std_pair_vec_;
  Eigen::Vector3d t_;
  Eigen::Matrix3d rot_;
};

class LoopResult {
public:
  LoopResult() { sol_res_.reset(new SolverResult()); };
  // 
  int frame_id_;

  // candidate_selector

  // candidate_verify
  std::shared_ptr<SolverResult> sol_res_;

  // final
  std::pair<int, double> loop_result_;
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform_;
  std::vector<std::pair<BTC, BTC>> loop_std_pair_;
};

class BtcDescManager {
 public:
  BtcDescManager() = default;

  ConfigSetting config_setting_;

  BtcDescManager(ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    loop_res_.reset(new LoopResult());
  };

  // if print debug info
  bool print_debug_info_ = false;

  // hash table, save all descriptors
  std::unordered_map<BTC_LOC, std::vector<BTC>> data_base_;

  // save all key clouds, optional
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_vec_;

  // save all binary descriptors of key frame
  std::vector<std::vector<BinaryDescriptor>> history_binary_list_;

  // save all planes of key frame, required
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> plane_cloud_vec_;

  /*debug*/
  // current proj plane
  std::shared_ptr<std::vector<std::shared_ptr<Plane>>> proj_plane_;
  std::shared_ptr<LoopResult> loop_res_;

  /*Three main processing functions*/

  // generate BtcDescs from a point cloud
  void GenerateBtcDescs(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                        const int frame_id, std::vector<BTC> &btcs_vec);

  // search result <candidate_id, plane icp score>. -1 for no loop
  void SearchLoop(const std::vector<BTC> &btcs_vec,
                  std::pair<int, double> &loop_result,
                  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
                  std::vector<std::pair<BTC, BTC>> &loop_std_pair);

  // add descriptors to database
  void AddBtcDescs(const std::vector<BTC> &btcs_vec);

  // Geometrical optimization by plane-to-plane icp
  void PlaneGeomrtricIcp(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);

 private:
  /*Following are sub-processing functions*/

  // voxelization and plane detection
  void init_voxel_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map);

  // acquire planes from voxel_map
  void get_plane(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud);

  void get_project_plane(
      std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
      std::vector<std::shared_ptr<Plane>> &project_plane_list);

  void merge_plane(std::vector<std::shared_ptr<Plane>> &origin_list,
                   std::vector<std::shared_ptr<Plane>> &merge_plane_list);

  // extract corner points from pre-build voxel map and clouds

  void binary_extractor(
      const std::vector<std::shared_ptr<Plane>> proj_plane_list,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
      std::vector<BinaryDescriptor> &binary_descriptor_list);

  void extract_binary(const Eigen::Vector3d &project_center,
                      const Eigen::Vector3d &project_normal,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::vector<BinaryDescriptor> &binary_list);

  // non maximum suppression, to control the number of corners
  void non_maxi_suppression(std::vector<BinaryDescriptor> &binary_list);

  // build BTCs from corner points.
  void generate_btc(const std::vector<BinaryDescriptor> &binary_list,
                    const int &frame_id, std::vector<BTC> &btc_list);

  // Select a specified number of candidate frames according to the number of
  // BtcDescs rough matches
  void candidate_selector(const std::vector<BTC> &btcs_vec,
                          std::vector<BTCMatchList> &candidate_matcher_vec);

  // Get the best candidate frame by geometry check
  void candidate_verify(
      BTCMatchList &candidate_matcher, double &verify_score,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
      std::vector<std::pair<BTC, BTC>> &sucess_match_vec);

  // Get the transform between a matched std pair
  void triangle_solver(std::pair<BTC, BTC> &std_pair, Eigen::Vector3d &t,
                       Eigen::Matrix3d &rot);

  // Get the transform between some matched std pairs
  void triangle_solver(std::vector<std::pair<BTC, BTC>> &std_pair_vec,
                       Eigen::Vector3d &t, Eigen::Matrix3d &rot,
                       std::shared_ptr<SolverResult> sol_res);

  // Geometrical verification by plane-to-plane icp threshold
  double plane_geometric_verify(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);
};

#endif  // BTC_H