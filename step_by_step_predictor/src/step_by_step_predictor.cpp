#include "step_by_step_predictor/step_by_step_predictor.h"

SBSPredictor::SBSPredictor():private_nh_("~")
{
  // param
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("mode", mode_, {0});
  private_nh_.param("visualize_observed_obs_poses", visualize_observed_obs_poses_, {false});
  private_nh_.param("visualize_predicted_obs_poses", visualize_predicted_obs_poses_, {false});
  private_nh_.param("sim_frame", sim_frame_, {"odom"});
  private_nh_.param("horizon", horizon_, {3.0});
  private_nh_.param("dt", dt_, {0.1});
  private_nh_.param("j_rate", j_rate_, {0.8});
  private_nh_.param("a_rate", a_rate_, {0.5});
  private_nh_.param("max_vel", max_vel_, {1.5});
  private_nh_.param("inc_tolerance", inc_tolerance_, {0.8});

  // subscriber
  sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &SBSPredictor::pedestrian_data_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher
  pub_predicted_states_ = nh_.advertise<std_msgs::Float32MultiArray>("/predicted_state", 1);

  // debug
  pub_observed_data_ = nh_.advertise<visualization_msgs::MarkerArray>("/observed_data", 1);
  pub_predicted_data_ = nh_.advertise<visualization_msgs::MarkerArray>("/predicted_data", 1);

}

// 歩行者データのコールバック関数
void SBSPredictor::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
{
  ped_states_.emplace(agents);
  flag_obs_data_ = true;
}

// 障害物データを取得
void SBSPredictor::get_obs_data()
{
  // ped_states_の配列のうち，1回のpublish分のデータ（配列の先頭の要素）のみ取得
  const auto people_states = ped_states_.front();

  // 保管場所が埋まっていたら，新しいデータ用の場所をあける
  if(is_store_obs_data_ == true)
  {
    for(int i=0; i<=max_id_; i++)
    {
      for(int j=1; j<mode_+2; j++)
      {
        obs_data_[i][j-1] = obs_data_[i][j];  // 古いデータを捨てる
      }
    }
  }

  // 最新データを格納 
  for(const auto& obs : people_states->agent_states)
  {
    int id = obs.id;
    Coordinate position;
    position.x = obs.pose.position.x;
    position.y = obs.pose.position.y;

    // 障害物情報を格納
    // 必要個数のデータがたまるまではセグフォ防止のためpush_backを使用して格納
    if(obs_data_store_counter_ == 0)
    {
      std::vector<Coordinate> positions;
      positions.push_back(position);
      obs_data_.push_back(positions);
    }
    else if(!(is_store_obs_data_))
    {
      obs_data_[id].push_back(position);
    }
    else
    {
      obs_data_[id][obs_data_store_counter_] = position;
    }

    // max_id_の更新
    if(id > max_id_)
      max_id_ = id;
  }

  // 障害物情報を必要数保管できているかの確認
  if(!(is_store_obs_data_))
  {
    obs_data_store_counter_++;
    
    if(obs_data_store_counter_ == mode_+2)
    {
      is_store_obs_data_ = true;
      obs_data_store_counter_ = mode_+2-1;  // 配列の番号に直す
    }
  }
}

// 観測データから障害物の速度を計算（加速度・ジャークも計算可能）
std::vector<Coordinate> SBSPredictor::calc_speed(const std::vector<Coordinate>& positions)
{
  std::vector<Coordinate> speeds;
  int size = positions.size();
  double dt = 1.0 / hz_;

  for(int i=1; i<size; i++)
  {
    Coordinate speed;
    speed.x = (positions[i].x - positions[i-1].x) / dt;
    speed.y = (positions[i].y - positions[i-1].y) / dt;
    speeds.push_back(speed);
  }

  return speeds;
}

// 将来時刻における加速度を計算
Coordinate SBSPredictor::calc_accel(const Coordinate tmp_a, const Coordinate j)
{
  Coordinate predict_a;

  // 加速し続けないようにジャークを減衰させる
  predict_a.x = tmp_a.x + ((1 - j_rate_) * j.x * dt_);
  predict_a.y = tmp_a.y + ((1 - j_rate_) * j.y * dt_);

  return predict_a;
}

// 将来時刻における速度を計算
Coordinate SBSPredictor::calc_velocity(const Coordinate tmp_v, const Coordinate a, const double max_speed)
{
  Coordinate predict_v;

  // 加速し続けないように加速度を減衰させる
  predict_v.x = tmp_v.x + ((1 - a_rate_) * a.x * dt_);
  predict_v.y = tmp_v.y + ((1 - a_rate_) * a.y * dt_);

  // 最大速度を超えてしまった場合は調整
  if(hypot(predict_v.x, predict_v.y) > max_speed)
  {
    const double dec_rate = hypot(predict_v.x, predict_v.y) / max_speed;

    predict_v.x = predict_v.x / dec_rate;
    predict_v.y = predict_v.y / dec_rate;
  }

  return predict_v;
}

// 障害物の移動先の座標を計算
Coordinate SBSPredictor::calc_position(const double tmp_x, const double tmp_y, const Coordinate v)
{
  Coordinate predict_position;

  predict_position.x = tmp_x + (v.x * dt_);
  predict_position.y = tmp_y + (v.y * dt_);

  return predict_position;
}

// 障害物の移動先の座標を計算
// Coordinate SBSPredictor::calc_position(const Coordinate current_position, const Coordinate v, const Coordinate a, const Coordinate j, const double dt)
// {
//   Coordinate predict_position;

//   predict_position.x = current_position.x + v.x*dt + a.x*dt*dt + j.x*dt*dt*dt;
//   predict_position.y = current_position.y + v.y*dt + a.y*dt*dt + j.y*dt*dt*dt;

//   return predict_position;
// }

// 障害物の移動後の速度を計算
// Coordinate SBSPredictor::calc_velocity(const Coordinate v, const Coordinate a, const Coordinate j, const double dt)
// {
//   Coordinate predict_vel;

//   predict_vel.x = v.x + a.x*dt + j.x*dt*dt;
//   predict_vel.y = v.y + a.y*dt + j.y*dt*dt;

//   return predict_vel;
// }

// 適切な角度(-M_PI ~ M_PI)を返す
double SBSPredictor::normalize_angle(double theta)
{
  if(theta > M_PI)
    theta -= 2.0 * M_PI;
  if(theta < -M_PI)
    theta += 2.0 * M_PI;

  return theta;
}

// 方位を計算
double SBSPredictor::calc_direction(const Coordinate vel)
{
  const double theta = atan2(vel.y, vel.x);

  return normalize_angle(theta);
}

// 移動予測
void SBSPredictor::predict_obs_states(std::vector< std::vector<Coordinate> >& predict_poses, std::vector< std::vector<State> >& predict_states)
{
  // 予測ステップ数を計算
  int predict_step = horizon_ / dt_;

    for(int i=0; i<=max_id_; i++)
    {
      // 1つの障害物に関するデータのみ格納
      std::vector<Coordinate> positions;
      for(int j=0; j<mode_+2; j++)
      {
        positions.push_back(obs_data_[i][j]);
      }

      // 速度を計算（mode0 ~ mode2共通）
      std::vector<Coordinate> v_list = calc_speed(positions);
      // 現在の速度を取得
      // back()を使用してvectorの末尾要素を取得
      Coordinate v = v_list.back();

      // 加速度を計算（mode1とmode2）
      std::vector<Coordinate> a_list;
      Coordinate a;
      if(mode_ >= 1)
      {
        a_list = calc_speed(v_list);
        // 現在の加速度を取得
        a = a_list.back();
        // a.x = a_list.back().x * dt_;
        // a.y = a_list.back().y * dt_;
      }
      else
      {
        a.x = 0.0;
        a.y = 0.0;
      }

      // ジャークを計算（mode2のみ）
      std::vector<Coordinate> j_list;
      Coordinate j;
      if(mode_ >= 2)
      {
        j_list = calc_speed(a_list);
        // 現在のジャークを取得
        j = j_list.back();
        // j.x = j_list.back().x * dt_;
        // j.y = j_list.back().y * dt_;
      }
      else
      {
        j.x = 0.0;
        j.y = 0.0;
      }

      // デバック用
    //   ROS_INFO_STREAM("id : " << i);
    //   ROS_INFO_STREAM("v : " << hypot(v.x, v.y));
    //   ROS_INFO_STREAM("a : " << hypot(a.x, a.y));
    //   ROS_INFO_STREAM("j : " << hypot(j.x, j.y));

      // 姿勢・速度予測
      std::vector<Coordinate> predict_positions;
      std::vector<State> predict_one_states;
      State predict_one_state;

      // 現在情報を格納
      predict_one_state.x = obs_data_[i].back().x;
      predict_one_state.y = obs_data_[i].back().y;
      predict_one_state.vel = hypot(v.x, v.y);
      predict_one_state.yaw = calc_direction(v);
      predict_one_states.push_back(predict_one_state);

      // 最高速度を増加許容範囲に基づき計算
      double max_speed = hypot(v.x, v.y);
      
      if(hypot(v.x, v.y) < max_vel_)
      {
        max_speed = hypot(v.x, v.y) + (inc_tolerance_ * (max_vel_ - hypot(v.x,v.y)));
      }

    //   ROS_INFO_STREAM("max_vel : " << max_speed);  // デバック用

      for(int k=1; k<=predict_step; k++)
      {
        // 予測時間を表示（デバック用）
        // double dt = dt_ * k;
        // ROS_INFO_STREAM("dt : " << dt);

        // 将来時刻における加速度を計算
        Coordinate predict_a = calc_accel(a, j);
        // ROS_INFO_STREAM("a : " << hypot(predict_a.x, predict_a.y));  // デバック用

        // 将来時刻における速度を計算
        Coordinate predict_vel = calc_velocity(v, predict_a, max_speed);
        // ROS_INFO_STREAM("v : " << hypot(predict_vel.x, predict_vel.y));  // デバック用

        // 移動先の座標を計算
        Coordinate predict_position = calc_position(predict_one_state.x, predict_one_state.y , predict_vel);
        predict_positions.push_back(predict_position);

        // Coordinate predict_position = calc_position(obs_data_[i].back(), v, a, j, dt);
        // predict_positions.push_back(predict_position);

        // 移動後の速度を計算
        // Coordinate predict_vel = calc_velocity(v, a, j, dt);
        
        // 予測データを格納
        j.x = (1 - j_rate_) * j.x;
        j.y = (1 - j_rate_) * j.y;
        a = predict_a;
        v = predict_vel;
        predict_one_state.x = predict_position.x;
        predict_one_state.y = predict_position.y;
        predict_one_state.vel = hypot(predict_vel.x, predict_vel.y);
        predict_one_state.yaw = calc_direction(predict_vel);
        predict_one_states.push_back(predict_one_state);
        // ROS_INFO_STREAM("vel : " << predict_one_state.vel);  // デバック用
      }

      predict_poses.push_back(predict_positions);
      predict_states.push_back(predict_one_states);
    }
}

// 予測した障害物情報をPublish
void SBSPredictor::publish_predict_data(const std::vector< std::vector<State> > obs_states)
{
  // 配列の行サイズを格納
  int row = obs_states.size();
  // 配列の列サイズを格納
  int column = obs_states[0].size();

  std_msgs::Float32MultiArray predict_data;

  // Publishするデータを格納
  for(int i=0; i<row; i++)
  {
    for(int j=0; j<column; j++)
    {
      predict_data.data.push_back(obs_states[i][j].x);
      predict_data.data.push_back(obs_states[i][j].y);
      predict_data.data.push_back(obs_states[i][j].vel);
      predict_data.data.push_back(obs_states[i][j].yaw);
    }
  }
  
  pub_predicted_states_.publish(predict_data);
}

// 障害物の位置情報を可視化
// visualization_msgs::MarkerArrayを使用
void SBSPredictor::visualize_obs_pose(const std::vector< std::vector<Coordinate> > obs_poses, ros::Publisher& pub_obs_pose, ros::Time now)
{
  // 配列の行サイズを格納
  int row = obs_poses.size();
  // 配列の列サイズを格納
  int column = obs_poses[0].size();

  // 配列サイズを表示（デバック用）
  ROS_INFO_STREAM("row : " << row);
  ROS_INFO_STREAM("column : " << column);

  visualization_msgs::MarkerArray obs_pose_lists;

  for(int i=0; i<row; i++)
  {
    visualization_msgs::Marker obs_pose_list;

    obs_pose_list.header.stamp = now;
    obs_pose_list.header.frame_id = sim_frame_;
    obs_pose_list.id = i;
    obs_pose_list.type = visualization_msgs::Marker::SPHERE_LIST;
    obs_pose_list.action = visualization_msgs::Marker::ADD;
    obs_pose_list.lifetime = ros::Duration();
    obs_pose_list.scale.x = 0.25;
    obs_pose_list.scale.y = 0.25;
    obs_pose_list.scale.z = 0.25;

    for(int j=0; j<column; j++)
    {
      geometry_msgs::Point obs_pose;
      obs_pose.x = obs_poses[i][j].x;
      obs_pose.y = obs_poses[i][j].y;
      obs_pose_list.points.push_back(obs_pose);

      std_msgs::ColorRGBA obs_pose_color;

      // 現在に近いほど色を濃くする
      if(column == mode_+2)  // 観測データ
      {
        obs_pose_color.r = 1.0;
        obs_pose_color.g = 0.0;
        obs_pose_color.b = 0.2;
        obs_pose_color.a = 0.2 + 0.8 * ((j+1) / (double)column);  // だんだん濃くなる
      }
      else                   // 予測データ
      {
        obs_pose_color.r = 0.0;
        obs_pose_color.g = 0.5;
        obs_pose_color.b = 1.0;
        obs_pose_color.a = 0.2 + 0.8 * (1 - (j/(double)column));  // だんだん薄くなる
      }

      obs_pose_list.colors.push_back(obs_pose_color);
    } 
    
    obs_pose_lists.markers.push_back(obs_pose_list);
  }

    pub_obs_pose.publish(obs_pose_lists); 
}

// 障害物情報を更新
void SBSPredictor::update_obs_data()
{
  ros::Time now = ros::Time::now();
  std::vector< std::vector<Coordinate> > predicted_obs_poses;
  std::vector< std::vector<State> > predicted_obs_states;

  // 障害物データを取得
  get_obs_data();

  // 障害物データが必要数たまってから処理を開始
  if(is_store_obs_data_)
  {
    predict_obs_states(predicted_obs_poses, predicted_obs_states);

    // 予測した障害物情報をPublish
    publish_predict_data(predicted_obs_states);

    // 障害物の観測位置を可視化
    if(visualize_observed_obs_poses_)
    {
      visualize_obs_pose(obs_data_, pub_observed_data_, now);
    }

    // 障害物の予測位置を可視化
    if(visualize_predicted_obs_poses_)
    {
      visualize_obs_pose(predicted_obs_poses, pub_predicted_data_, now);
    }
  }

  // ped_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
  // これをしないと，front() でデータを取得する際，同じデータしか取得できない
  ped_states_.pop();
}

// メイン文で実行する関数
void SBSPredictor::process()
{
  ros::Rate loop_rate(hz_);

  while(ros::ok())
  {
    if(flag_obs_data_)
    {
        update_obs_data();
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_obs_data_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}