#include "dwa_planner.h"


DWAPlanner::DWAPlanner(State init_state)
{
  x_ = init_state;
  goal_ = Point({{init_state[0], init_state[1]}});
  u_ = Control({{0.0, 0.0}});
  trajectory_.push_back(x_);
}

//! Motion function
    /*!
      Executes a motion simulation step to predict robot's state
      \param x State of the robot
      \param u Motion cmd
      \param dt Time delta
      \return State after executing a motion step from the state x with motion cmd u
    */

State DWAPlanner::Motion(State x, double current_x, double current_y, double heading)
{
  x[0] = current_x; // x 위치 업데이트
  x[1] = current_y; // y 위치 업데이트
  x[2] = heading; // 각도 업데이트

   // 선형 속도 계산
  float distance_to_goal = std::sqrt(pow(goal_[0] - current_x, 2) + pow(goal_[1] - current_y, 2));
  float linear_speed = std::min(config_.max_speed, distance_to_goal / config_.predict_time); // 최대 속도와 목표 거리 기반

  x[3] = linear_speed; // 선형 속도 업데이트

  x[4] = 0; // 각속도

  u_ = Control({{linear_speed, x[4]}});

  x_ = x;

  return x;
}

//! Motion function
    /*!
      Calculates the dynamic window depending on constrains defined in Config class
      \return window of possible speeds depending on config parameters
    */
Window DWAPlanner::CalcDynamicWindow()
{
  return {{std::max((u_[0] - config_.max_accel * config_.dt), config_.min_speed),
           std::min((u_[0] + config_.max_accel * config_.dt), config_.max_speed),
           std::max((u_[1] - config_.max_dyawrate * config_.dt), -config_.max_yawrate),
           std::min((u_[1] + config_.max_dyawrate * config_.dt), config_.max_yawrate)}};
}

//! CalcTrajectory function
    /*!
      Calculates trajectory followed with speed and yaw_rate
      /param v speed
      /param y yaw_rate
      \return trajectory predicted when moving with speeds v and y 
              on a time frime (predict_time) 
    */
Trajectory DWAPlanner::CalcTrajectory(float v, float y)
{
  State x(x_);             // Copy of current state
  Trajectory traj;
  traj.push_back(x);       // First element of the trajectory is current state
  float time = 0.0;
  while (time <= config_.predict_time)  //Simulate trajectory for predict_time seconds
  {
    x = Motion(x, x[0], x[1], x[2]);  // Perform one motion step
    traj.push_back(x);
    time += config_.dt;
  }
  return traj;
}

float DWAPlanner::CalcObstacleCost(Trajectory traj)
{
    float minr = std::numeric_limits<float>::max();
    int skip_n = 1; // skip some points of the trajectory

    for (unsigned int ii = 0; ii < traj.size(); ii += skip_n)
    {
        for (const auto& obstacle : ob_)
        {
            float dx = traj[ii][0] - obstacle.x; // x 거리
            float dy = traj[ii][1] - obstacle.y; // y 거리

            float r = std::sqrt(dx * dx + dy * dy); // 2D 거리 계산
            if (r <= config_.robot_radius)
            {
                return std::numeric_limits<float>::max(); // 충돌 감지
            }

            if (minr >= r)
            {
                minr = r; // 최소 거리 업데이트
            }
        }
    }
    return 1.0 / minr; // 최소 거리의 역수 반환
}


//! CalcToGoalCost function
    /*!
      Calculate cost depending on distance to goal of the current trajectory
      /param traj Trajectory that we want to calculate the cost of
      \return cost float to goal cost
    */
float DWAPlanner::CalcToGoalCost(Trajectory traj)
{

    // Cost is defined by the angle to goal
    float goal_magnitude = std::sqrt(pow(goal_[0]-x_[0], 2) + pow(goal_[1]-x_[1], 2));
    float traj_magnitude = std::sqrt(pow(traj.back()[0]-x_[0], 2) + pow(traj.back()[1]-x_[1], 2));

    // 0으로 나누기 방지
    if (goal_magnitude == 0 || traj_magnitude == 0) {
        return std::numeric_limits<float>::max(); // 비용을 매우 큰 값으로 설정
    }

    float dot_product = ((goal_[0]-x_[0]) * (traj.back()[0]-x_[0])) + ((goal_[1]-x_[1]) * (traj.back()[1]-x_[1]));
    float error = dot_product / (goal_magnitude * traj_magnitude);

    // error를 -1과 1 사이로 제한
    error = std::max(-1.0f, std::min(1.0f, error));

    float error_angle = std::acos(error);
    float cost = config_.to_goal_cost_gain * error_angle;

    return cost;
}


//! CalcFinalInput function
    /*!
      Calculate motion command by evaluating the cost of trajectories inside window
      /param dw Dynamic window
      \return trajectory best trajectory
    */
Trajectory DWAPlanner::CalcFinalInput(Window dw)
{
  float min_cost = 10000.0;
  Control min_u = u_;
  min_u[0] = 0.0;
  Trajectory best_traj;

  std::cout << dw[0] << " " << dw[1] << " " << dw[2] << " " << dw[3] << "\n";

  // evalucate all trajectory with sampled input in dynamic window
  for (float v = dw[0]; v <= dw[1]; v += config_.v_reso)
  {
    for (float y = dw[2]; y <= dw[3]; y += config_.yawrate_reso)
    {
      Trajectory traj = CalcTrajectory(v, y);

      // Add all costs
      float to_goal_cost = CalcToGoalCost(traj);
      float dist_to_goal = sqrt(pow((goal_[0] - x_[0]),2) + pow((goal_[1] - x_[1]), 2));
      dist_to_goal = dist_to_goal < 1.0f ? dist_to_goal : 1.0f;
      float speed_cost = dist_to_goal*config_.speed_cost_gain * (config_.max_speed - traj.back()[3]);
      float ob_cost = 1.0*CalcObstacleCost(traj);
      float final_cost = to_goal_cost + speed_cost + ob_cost;

      // Save motion command that produces smallest cost
      if (min_cost >= final_cost)
      {
        min_cost = final_cost;
        min_u = Control{{v, y}};
        best_traj = traj;

        std::cout << "min cost" << "\n";
      }
    }
  }

  u_ = min_u;
  return best_traj;
}

//! DWAControl function
    /*!
      Calculate dynamic window and trajectory with the smallest cost
      \return trajectory best trajectory
    */
Trajectory DWAPlanner::DWAControl()
{
  // # Dynamic Window control
  Window dw = CalcDynamicWindow();
  Trajectory traj = CalcFinalInput(dw);
  return traj;
}

//! SetObstacles function
    /*!
      Stores obstacle points from scan msgs
      \param scan_distances Vector with the scan distances
      \param angle_increment Angle increment for each scan point in the scan_distances vector
      \param angle_min Minimum angle of the scan points
      \param angle_max Maximum angle of the scan points
      \param range_min Min posible distance possible in the scan distances
      \param range_max Max posible distance possible in the scan distances
    */

void DWAPlanner::SetObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidar_points, float range_min, float range_max)
{
    ob_.clear(); // 장애물 벡터 초기화
    float min_dist = std::numeric_limits<float>::max();
    pcl::PointXYZ closest_obstacle(min_dist, min_dist, min_dist); // 초기화


    // Calculate the closest obstacle
    for (const auto& point : lidar_points->points) // PCL 포인트 클라우드에서 포인트 접근
    {
        // 장애물의 거리 계산
        float distance = std::sqrt(pow(x_[0] - point.x, 2) +  pow(x_[1] - point.y, 2));

        // 범위 필터링
        if (distance < range_max && distance > range_min)
        {
            // 가장 가까운 장애물 업데이트
            if (distance < min_dist)
            {
                min_dist = distance;
                closest_obstacle = point; // 가장 가까운 장애물 위치 저장
            }
        }
    }

    // 가장 가까운 장애물 저장
    if (min_dist < std::numeric_limits<float>::max())
    {
        ob_.push_back(closest_obstacle);
        if (ob_.size() >= 25)
            ob_.erase(ob_.begin()); // 오래된 장애물 정보 제거
    }
}



//! SetState function
    /*!
      Stores the current state of the robot for planning
      \param state 
    */
void DWAPlanner::SetState(State state)
{
  x_ = state;
  //trajectory_.push_back(x);

}

//! SetGoal function
    /*!
      Stores the desired goal 
      \param goal 
    */
void DWAPlanner::SetGoal(Point goal){
  goal_ = goal;
  goal_reached_ = false;
}


//! GetCmd function
    /*!
      Returns speed and yaw_rate commands defined by the DWA
    */
Control DWAPlanner::GetCmd()
{
  trajectory_ = DWAControl();
  //when goal is reached
  if(sqrt(pow(goal_[0]-x_[0],2) + pow(goal_[1]-x_[1],2))<0.5){
    u_ = Control{{0, 0.0}};
    if(!goal_reached_)
      goal_reached_ = true;
  }
  return u_;
}

void DWAPlanner::PublishTrajectory(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher) {
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  
  marker.header.frame_id = "map"; // 또는 "odom"
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 선으로 표시
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 1.0; // 선의 두께
  marker.color.r = 1.0; // 빨간색
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0; // 불투명
  
  // 로컬 경로에 따른 포인트 추가
  for (const auto& state : trajectory_) {
      geometry_msgs::msg::Point p;
      p.x = state[0]; // x 좌표
      p.y = state[1]; // y 좌표
      p.z = 0; // 2D 경로이므로 z는 0
      marker.points.push_back(p);
      std::cout << "로컬 경로: " << p.x << " " <<p.y << "\n";
  }

  marker_array.markers.push_back(marker);
  publisher->publish(marker_array);
}
