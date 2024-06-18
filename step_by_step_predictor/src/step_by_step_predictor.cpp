#include "step_by_step_predictor/step_by_step_predictor.h"

SBSPredictor::SBSPredictor():private_nh_("~")
{
  // param
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("mode", mode_,{0});
//   private_nh_.param("visualize_current_people_poses", visualize_current_people_poses_, {false});
//   private_nh_.param("visualize_selected_current_people_poses", visualize_selected_current_people_poses_, {false});
//   private_nh_.param("visualize_future_people_poses", visualize_future_people_poses_, {false});
//   private_nh_.param("flag_prediction", flag_prediction_, {true});
  private_nh_.param("sim_frame", sim_frame_, {"odom"});
//   private_nh_.param("robot_frame", robot_frame_, {"odom"});
//   private_nh_.param("people_frame", people_frame_, {"base_footprint"});
//   private_nh_.param("consider_dist_border", consider_dist_border_, {8.0});
//   private_nh_.param("predict_dist_border", predict_dist_border_, {3.0});
//   private_nh_.param("tmp_robot_x", tmp_robot_x_, {0.0});
//   private_nh_.param("tmp_robot_y", tmp_robot_y_, {0.0});

  // subscriber
  sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &SBSPredictor::pedestrian_data_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher

  // debug
  pub_observed_data_ = nh_.advertise<visualization_msgs::MarkerArray>("/observed_data", 1);

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

// 障害物の位置情報を可視化
// visualization_msgs::MarkerArrayを使用
void SBSPredictor::visualize_obs_pose(const std::vector< std::vector<Coordinate> > obs_poses, const ros::Publisher& pub_obs_pose, ros::Time now)
{
  // 配列の行サイズを格納
  int row = obs_poses.size();
  // 配列の列サイズを格納
  int column = obs_poses[0].size();

  // 配列サイズを表示
  ROS_INFO_STREAM("row : " << row);
  ROS_INFO_STREAM("column : " << column);

//   for(int i=0; i<row; i++)
//   {
//     visualization_msgs::MarkerArray obs_pose_lists;
    
//     for(int j=0; j<column; j++)
//     {
//       visualization_msgs::Marker obs_pose_list;

//       obs_pose_list.header.stamp = now;
//       obs_pose_list.header.frame_id = sim_frame_;
//       obs_pose_list.id = i;
//       obs_pose_list.type = visualization_msgs::Marker::SPHERE_LIST;
//       obs_pose_list.action = visualization_msgs::Marker::ADD;
//       obs_pose_list.lifetime = ros::Duration();
//       obs_pose_list.scale.x = 0.25;
//       obs_pose_list.scale.y = 0.25;
//       obs_pose_list.scale.z = 0.25;
//       obs_pose_list.pose.position.x = obs_poses[i][j].x;
//       obs_pose_list.pose.position.y = obs_poses[i][j].y;
//       obs_pose_list.color.r = 0.0;
//       obs_pose_list.color.g = 0.5;
//       obs_pose_list.color.b = 1.0;

//       // 現在に近いほど色を濃くする
//       if(column == mode_+2)
//       {
//         obs_pose_list.color.a = 0.2 + 0.8 * ((j+1) / column);  // だんだん濃くなる
//       }
//       else
//       {
//         obs_pose_list.color.a = 0.2 + 0.8 * (1 - (j/column));  // だんだん薄くなる
//       }
      
//       obs_pose_lists.markers.push_back(obs_pose_list);
//     }

//     pub_obs_pose.publish(obs_pose_lists);
//   }

  // aaaaaaaaaaaaaaaaaaaaaaaaaa

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
        obs_pose_color.a = 0.2 + 0.8 * ((j+1) / column);  // だんだん濃くなる
      }
      else                   // 予測データ
      {
        obs_pose_color.r = 0.0;
        obs_pose_color.g = 0.5;
        obs_pose_color.b = 1.0;
        obs_pose_color.a = 0.2 + 0.8 * (1 - (j/column));  // だんだん薄くなる
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

  // 障害物データを取得
  get_obs_data();

  if(is_store_obs_data_)
  {
    visualize_obs_pose(obs_data_, pub_observed_data_, now);
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