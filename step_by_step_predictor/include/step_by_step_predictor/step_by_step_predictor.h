#ifndef STEP_BY_STEP_PREDICTOR_H
#define STEP_BY_STEP_PREDICTOR_H

#include <ros/ros.h>
#include <queue>
// #include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2/utils.h>

// original_msgs
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
// #include <pedestrian_msgs/PersonState.h>
// #include <pedestrian_msgs/PeopleStates.h>

// ===== 構造体 =====
struct Coordinate
{
    double x;  // [m]
    double y;  // [m]
};

struct State
{
    double x;    // [m]
    double y;    // [m]
    double vel;  // [m/s]
    double yaw;  // [rad]
};

// ===== クラス =====
class SBSPredictor
{
public:
  SBSPredictor();
  void process();

private:
  // コールバック関数
  void pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents);

  // 引数あり関数
  std::vector<Coordinate> calc_speed(const std::vector<Coordinate>& positions);                                                        // 障害物の速度を計算（加速度・ジャークも計算可能）
  Coordinate calc_accel(const Coordinate tmp_a, const Coordinate j);                                                                      // 将来時刻における加速度を計算
  Coordinate calc_velocity(const Coordinate tmp_v, const Coordinate a, const double max_speed);                                           // 将来時刻における速度を計算
  Coordinate calc_position(const double tmp_x, const double tmp_y, const Coordinate v);                 // 障害物の移動先の座標を計算
  // Coordinate calc_position(const Coordinate current_position, const Coordinate v, const Coordinate a, const Coordinate j, const double dt);  // 障害物の移動先の座標を計算
  // Coordinate calc_velocity(const Coordinate v, const Coordinate a, const Coordinate j, const double dt);                                     // 障害物の移動後の速度を計算
  double normalize_angle(double theta);                                                                                                      // 適切な角度(-M_PI ~ M_PI)を返す
  double calc_direction(const Coordinate vel);                                                                                               // 方位を計算
  void predict_obs_states(std::vector< std::vector<Coordinate> >& predict_poses, std::vector< std::vector<State> >& predict_states);         // 移動予測
  void publish_predict_data(const std::vector< std::vector<State> > obs_states);                                                      // 予測した障害物情報をPublish
  void visualize_obs_pose(const std::vector< std::vector<Coordinate> > obs_poses, ros::Publisher& pub_obs_pose, ros::Time now);  // 障害物の位置情報を可視化

  // 引数なし関数
  void get_obs_data();     // 障害物データを取得
  void update_obs_data();  // 障害物情報を更新

  // yamlファイルで設定可能な変数
  int hz_;                    // ループ周波数 [Hz]
  int mode_;                    // 予測モード→0：等速モデル，1：加速度を考慮，2：ジャークを考慮
  bool visualize_observed_obs_poses_;   // 障害物の観測位置を可視化するかの設定用
  bool visualize_predicted_obs_poses_;  // 障害物の予測位置を可視化するかの設定用
  std::string sim_frame_;       // シミュレーターからの歩行者の位置情報のframe_id
  double horizon_;                      // 予測ホライズン [s]
  double dt_;                           // 微小時間 [s]
  double j_rate_;                       // ジャークの減速比率
  double max_vel_;                      // 障害物の最高速度
  double inc_tolerance_;                // 最高速度に対する増加許容範囲

  // その他の変数
  int max_id_ = 0;  // 障害物情報のidの最大値格納用
      
  // msgの受け取り判定用
  bool flag_obs_data_ = false;

  // 障害物情報を必要数保管できているかの確認用
  int obs_data_store_counter_ = 0;
  bool is_store_obs_data_ = false;

  // 障害物情報格納用
  std::vector< std::vector<Coordinate> > obs_data_;

  // NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_ped_states_;

  // Publisher
  ros::Publisher pub_predicted_states_;
  ros::Publisher pub_observed_data_;
  ros::Publisher pub_predicted_data_;


  // 各種オブジェクト
  std::queue<pedsim_msgs::AgentStatesConstPtr> ped_states_;  // 歩行者情報
};

#endif  // STEP_BY_STEP_PREDICTOR_H