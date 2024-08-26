#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <math.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <limits>
#include <cmath>

#include <std_msgs/msg/float64_multi_array.hpp>


#define PI 3.141592653


using Trajectory = std::vector<std::array<float, 5>>;
using Obstacle = std::vector<std::array<float, 2>>;
using State = std::array<float, 5>; // x, y,  각도 (orientation) (라디안), 선형 속도, 각속도
using Window = std::array<float, 4>;
using Point = std::array<float, 2>;
using Control = std::array<float, 2>;

/*! Configuration class with the parameters of the algorithm */
class Config{
public:
  float max_speed = 0.4;   // m/s
  float min_speed = -0.2;
  float max_yawrate = 60.0 * PI / 180.0;  // 로봇의 최대 회전 속도 rad/s
  float max_accel = 0.3;  // 로봇의 최대 가속도 m/s^2
  float robot_radius = 0.45;  // 로봇의 반지름 m
  float max_dyawrate = 60.0 * PI / 180.0; // 로봇의 최대 각속도 rad/s

  float v_reso = 0.01;  // 속도 해상도 m/s
  float yawrate_reso = 0.1 * PI / 180.0;  // 각속도 해상도를 나타내며, 로봇의 각속도를 샘플링하는 간격을 정의 rad/s

  float dt = 0.1; // 시간 간격 s
  float predict_time = 3.5; // 로봇의 동작을 예측 s
  float to_goal_cost_gain = 0.75; // 목표까지의 거리 비용을 계산
  float speed_cost_gain = 1.5; // 속도 비용을 계산
};

/*! DWAPlanner class */
class DWAPlanner {
    public:
        DWAPlanner(State init_state);
        // void SetObstacles(std::vector<float> scan_distances, float angle_increment, float angle_min, 
        //                   float angle_max, float range_min, float range_max); //!< Stores obstacle points from scan msgs
        
        void SetObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidar_points, float range_min, float range_max);
      
        void SetState(State state);       //!< Stores the current state of the robot for planning
        void SetGoal(Point goal);   /*!< Motion command (speed and yaw rate) defined by DWA */
        // Obstacle ob_;                   /*!< Vector with scan points corresponding to obstacles*/
        // void poseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        
        
        std::vector<pcl::PointXYZ> ob_; // 장애물 저장 벡터

        
        Trajectory trajectory_;           //!< Returns current trajectory to be followed by the robot according to the DWA
        Control GetCmd();                 //!< Returns speed and yaw_rate commands defined by the DWA


        State Motion(State x, Control u, float dt);   //!< Executes a motion simulation step to predict robot's state
        Window CalcDynamicWindow();                   //!< Calculates the dynamic window depending on constrains defined in Config class
        Trajectory CalcTrajectory(float v, float y);  //!< Calculates trajectory followed with speed and yaw_rate
        float CalcObstacleCost(Trajectory traj);      //!< Calculate obstacle cost defined by the current trajectory
        float CalcToGoalCost(Trajectory traj);        //!< Calculate cost depending on distance to goal of the current trajectory
        Trajectory CalcFinalInput(Window dw);         //!< Calculate motion command by evaluating the cost of trajectories inside window
        Trajectory DWAControl();                      //!< Calculate dynamic window and trajectory with the smallest cost




        State x_;                       /*!< Vector containing state of the robot (position and velocities) */
        Point goal_;                    /*!< Vector containing x and y coordinates of the current goal */
        Control u_;                     /*!< Motion command (speed and yaw rate) defined by DWA */
       
        int count_ = 0;
        Config config_;
        bool goal_reached_ = false;     /*!< Bool that defines if the goal has been reached */

        void PublishTrajectory(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher);
};
