#ifndef STEP_BY_STEP_PREDICTOR_H
#define STEP_BY_STEP_PREDICTOR_H

#include <ros/ros.h>
#include <queue>
// #include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/TransformStamped.h>
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
  void visualize_obs_pose(const std::vector< std::vector<Coordinate> > obs_poses, const ros::Publisher& pub_obs_pose, ros::Time now);  // 障害物の位置情報を可視化
  // void get_ped_data(pedestrian_msgs::PeopleStates& current_people, ros::Time now);                                                               // 歩行者データを取得
  // double calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y);                       // ロボットと歩行者の間の距離を計算
  // void transform_and_calc_speed(const pedestrian_msgs::PersonState& current_person, pedestrian_msgs::PersonState& selected_current_person, const double after_ones_x, const double after_ones_y);   // 歩行者情報をodomからbase_footprintに変更 & base_link座標系での速度を計算
  // double calc_direction(const double robot_x, const double robot_y, const double person_x, const double person_y);                                               // 方位を計算
  // double normalize_angle(double theta);                                           // 適切な角度(-M_PI ~ M_PI)を返す
  // void transform_ped_pose(const double before_x, const double before_y,  pedestrian_msgs::PersonState& future_person);                                             // 歩行者情報をodomからbase_footprintに変更
  // void predict_future_ped_states(const pedestrian_msgs::PeopleStates& current_people, pedestrian_msgs::PeopleStates& future_people, pedestrian_msgs::PeopleStates& selected_current_people, ros::Time now);  // 歩行者の将来位置を予測
  // void visualize_people_pose(const pedestrian_msgs::PeopleStates& people, const ros::Publisher& pub_people_poses, ros::Time now);                   // 歩行者の位置情報を可視化

  // 引数なし関数
  void get_obs_data();     // 障害物データを取得
  void update_obs_data();  // 障害物情報を更新

  // yamlファイルで設定可能な変数
  int hz_;                    // ループ周波数 [Hz]
  int mode_;                    // 予測モード→0：等速モデル，1：加速度を考慮，2：ジャークを考慮
  // bool visualize_current_people_poses_;     // 歩行者の現在位置（odom）を可視化するかの設定用
  // bool visualize_selected_current_people_poses_;  // ロボットに近い歩行者の現在位置（base_footprint）を可視化するかの設定用
  // bool visualize_future_people_poses_;      // 予測した歩行者の将来位置を可視化するかの設定用
  // bool flag_prediction_;              // 歩行者の移動先予測をするかの変更用
  std::string sim_frame_;       // シミュレーターからの歩行者の位置情報のframe_id
  // std::string robot_frame_;        // ロボットの位置情報のframe_id
  // std::string people_frame_;            // 歩行者の位置情報のframe_id
  // double consider_dist_border_;       // cost_map_createrにデータを渡す歩行者の距離 [m]
  // double predict_dist_border_;          // 歩行者の将来位置を予測する距離 [m]

  // その他の変数
  int max_id_ = 0;  // 障害物情報のidの最大値格納用
      
  // msgの受け取り判定用
  bool flag_obs_data_ = false;

  // 障害物情報を必要数保管できているかの確認用
  int obs_data_store_counter_ = 0;
  bool is_store_obs_data_ = false;

  // 座標変換の判定用
  // bool flag_frame_change_ = true;

  // 歩行者情報格納用
  std::vector< std::vector<Coordinate> > obs_data_;

  // NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_ped_states_;
  // ros::Subscriber sub_robot_odom_;

  // Publisher
  // ros::Publisher pub_predicted_obs_poses_;
  // ros::Publisher pub_selected_current_ped_poses_;
  // ros::Publisher pub_selected_current_people_states_;
  // ros::Publisher pub_future_ped_poses_;
  ros::Publisher pub_observed_data_;

  // tf
  // tf2_ros::Buffer tf_buffer_;

  // 各種オブジェクト
  std::queue<pedsim_msgs::AgentStatesConstPtr> ped_states_;  // 歩行者情報
};

#endif  // STEP_BY_STEP_PREDICTOR_H