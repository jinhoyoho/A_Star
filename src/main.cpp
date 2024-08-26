#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "a_star.h"
#include "nodes.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cstddef>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav2_core/controller.hpp>
#include <dwb_core/dwb_local_planner.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "dwa_planner.h"
#include <cmath>
#include <pcl/point_cloud.h>
#include <vector>
#include <limits>


rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr node_publisher_;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr multi_array_publisher_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;

std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
std::unique_ptr<global_planner::AStar> a_star_;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;


// 경로를 저장할 변수 추가
std::vector<Node> global_path;
bool path_computed = false; // 경로가 계산되었는지 여부를 확인하는 플래그
bool arrive_flag = false; // 도착 플래그

double wx, wy;  // 월드 좌표
double origin_x = -250;
double origin_y = -150;  // 원점 좌표

double current_x, current_y; // 현재 x, y 좌표
double heading; // 헤딩 값
double ld_x, ld_y; // ld x, y 좌표

Node start((-1)*origin_x + current_x, (-1)*origin_y + current_y);
Node goal(150, 150); // 해상도에 맞춰 조정

State init_state = {start.x(), start.y(), 0, 0, 0};


DWAPlanner dwa(init_state); // 초기 DWA 생성

size_t goal_index = 0;


void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    current_x = msg->data[0];   // 현재 x좌표
    current_y = msg->data[1];   // 현재 y좌표
    heading = msg->data[2];     // 현재 헤딩값

    start.set_x((-1)*origin_x + current_x);
    start.set_y((-1)*origin_y + current_y);

    std::cout << current_x << " " << current_y << " " << heading << "\n";
    
}

// Static Transform Publisher 추가
void publishStaticTransform() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Clock().now();
    transform.header.frame_id = "odom"; // 부모 프레임
    transform.child_frame_id = "map"; // 자식 프레임
    transform.transform.translation.x = 0.0; // 변환의 x 좌표
    transform.transform.translation.y = 0.0; // 변환의 y 좌표
    transform.transform.translation.z = 0.0; // 변환의 z 좌표
    transform.transform.rotation.x = 0.0; // 회전의 x 좌표
    transform.transform.rotation.y = 0.0; // 회전의 y 좌표
    transform.transform.rotation.z = 0.0; // 회전의 z 좌표
    transform.transform.rotation.w = 1.0; // 회전의 w 좌표

    static_broadcaster_->sendTransform(transform);
}

void visualizePath(const std::vector<Node>& path) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1; // 선의 두께
    marker.color.a = 1.0; // 불투명도

    // 색상 설정: 파란색
    marker.color.r = 0.0; // 빨간색
    marker.color.g = 0.0; // 초록색
    marker.color.b = 1.0; // 파란색

    for (const auto& node : path) {
        geometry_msgs::msg::Point p;
        p.x = node.x(); // Node 클래스의 x() 메서드 사용
        p.y = node.y(); // Node 클래스의 y() 메서드 사용
        p.z = 0;


        marker.points.push_back(p);
    }

    path_publisher_->publish(marker);
}


void visualizeStartAndGoal(const Node& start, const Node& goal) {
    visualization_msgs::msg::Marker start_marker;
    start_marker.header.frame_id = "map";
    start_marker.header.stamp = rclcpp::Clock().now();
    start_marker.ns = "start";
    start_marker.id = 1;
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.scale.x = 1; // 크기
    start_marker.scale.y = 1; // 크기
    start_marker.scale.z = 0; // 크기
    start_marker.color.a = 1.0; // 불투명도
    start_marker.color.g = 1.0; // 초록색
    start_marker.color.r = 0.0; // 빨간색
    start_marker.color.b = 0.0; // 파란색

    // 시작점 위치 설정
    start_marker.pose.position.x = start.x();
    start_marker.pose.position.y = start.y();
    start_marker.pose.position.z = 0;

    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = rclcpp::Clock().now();
    goal_marker.ns = "goal";
    goal_marker.id = 2;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.scale.x = 1; // 크기
    goal_marker.scale.y = 1; // 크기
    goal_marker.scale.z = 1; // 크기
    goal_marker.color.a = 1.0; // 불투명도
    goal_marker.color.r = 1.0; // 빨간색
    goal_marker.color.g = 0.0; // 초록색
    goal_marker.color.b = 0.0; // 파란색

    // 목표점 위치 설정
    goal_marker.pose.position.x = goal.x();
    goal_marker.pose.position.y = goal.y();
    goal_marker.pose.position.z = 0;

    // 마커 퍼블리시
    node_publisher_->publish(start_marker);
    node_publisher_->publish(goal_marker);
}


void loadPointCloudFromPCD(const std::string& file_path) {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, cloud) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not read PCD file");
        return;
    }

    unsigned int mx, my;
    for (const auto& point : cloud) {
        // 월드 좌표를 맵 좌표로 변환
        if (costmap_->worldToMap(point.x*3, point.y*3, mx, my)) {
            // 해당 위치에 장애물 설정
            costmap_->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
            
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "Point (%f, %f) could not be mapped to costmap. "
                "Costmap Origin: (%f, %f), Size: (%u, %u)",
                point.x, point.y,
                costmap_->getOriginX(), costmap_->getOriginY(),
                costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
        }
    }

    std::cout << "PCD complete." << "\n";

}

void publishCostmap() {
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.header.stamp = rclcpp::Clock().now();
    occupancy_grid.info.resolution = costmap_->getResolution();

    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();

    // std::cout << "셀의 개수: " << size_x << ", " << size_y << "\n";

    occupancy_grid.info.width = size_x;
    occupancy_grid.info.height = size_y;

    // 셀 좌표 (0, 0)에 해당하는 실제 월드 좌표를 계산하여 wx와 wy에 저장
    // costmap_->mapToWorld(0, 0, wx, wy);
    
    occupancy_grid.info.origin.position.x = 0;
    occupancy_grid.info.origin.position.y = 0;
    occupancy_grid.info.origin.position.z = 0;
    occupancy_grid.info.origin.orientation.w = 1.0;

    occupancy_grid.data.resize(size_x * size_y);

    for (unsigned int i = 0; i < size_x; i++) {
        for (unsigned int j = 0; j < size_y; j++) {
            occupancy_grid.data[j * size_x + i] = costmap_->getCost(i, j);
        }
    }

    costmap_publisher_->publish(occupancy_grid);
}

void publish_float64_multiarray() {
    // 발행할 메시지 생성
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {ld_x, ld_y, static_cast<double>(arrive_flag)}; // 데이터 초기화

    // 메시지 발행
    multi_array_publisher_->publish(msg);
}

void findClosestNode(std::vector<Node>& path, size_t& start_index) {
    double min_distance = std::numeric_limits<double>::max(); // 초기 최소 거리
    size_t closest_index = start_index; // 가장 가까운 노드의 인덱스 초기화

    // start_index부터 시작하여 path를 순회
    for (size_t i = start_index; i < path.size(); ++i) {
        const auto& node = path[i];
        double distance = std::sqrt(std::pow(node.x() - start.x(), 2) + std::pow(node.y() - start.y(), 2));
        
        // 최소 거리 갱신
        if (distance < min_distance) {
            min_distance = distance;
            ld_x = node.x();
            ld_y = node.y();
            closest_index = i; // 가장 가까운 노드의 인덱스 저장
        }
    }

    // 가장 가까운 노드 인덱스를 다음 검색을 위해 업데이트
    start_index = closest_index + 1; // 다음 검색을 위해 인덱스를 업데이트

    // std::cout << "ld: " << ld_x << " " << ld_y << "\n";

    publish_float64_multiarray(); // publish
}


// dwa 관련 함수
void SetDWA()
{
    Point goal_point = {ld_x, ld_y};

    dwa.SetGoal(goal_point);    // 목적지 설정
    
    std::cout << ld_x << " " << ld_y << "\n";

    // PCL 포인트 클라우드 객체 생성
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points(new pcl::PointCloud<pcl::PointXYZ>);

    // // 샘플 3D LIDAR 데이터 추가
    // lidar_points->points.push_back(pcl::PointXYZ(1.0f, 2.0f, 0.0f)); // 유효한 장애물
    // lidar_points->points.push_back(pcl::PointXYZ(3.0f, -1.0f, 0.0f)); // 유효한 장애물
    // lidar_points->points.push_back(pcl::PointXYZ(0.1f, 0.1f, 0.0f)); // 너무 가까운 장애물 (0.1m)
    // lidar_points->points.push_back(pcl::PointXYZ(6.0f, 6.0f, 0.0f)); // 너무 먼 장애물 (6.0m)
    // lidar_points->points.push_back(pcl::PointXYZ(-2.0f, 4.0f, 0.0f)); // 유효한 장애물
    // 포인트 클라우드의 크기 설정
    lidar_points->width = lidar_points->points.size();
    lidar_points->height = 1; // 단일 행의 포인트 클라우드


    // float angle_increment = M_PI / 180.0; // 1도 단위 각도 증가
    // float angle_min = -M_PI / 4; // -45도
    // float angle_max = M_PI / 4; // 45도

    float range_min = 0.2; // 최소 거리
    float range_max = 5.0; // 최대 거리

    // 장애물 정보 업데이트
    dwa.SetObstacles(lidar_points, range_min, range_max);

    // 로봇의 제어 명령을 얻고 출력
    
    Control command = dwa.GetCmd(); // DWAPlanner에서 명령 얻기

    // 현재 명령 출력
    // std::cout << " | Linear Velocity: " << command[0] 
    //           << " | Angular Velocity: " << command[1] << std::endl;

    // 로봇 상태 업데이트 (모션 적용)
    init_state = dwa.Motion(init_state, command, 0.1); // dt = 0.1초
    dwa.SetState(init_state); // 업데이트된 상태 설정

    dwa.PublishTrajectory(trajectory_publisher_);

    // 목표에 도달했는지 확인
    // if (sqrt(pow(goal_point[0] - init_state[0], 2) + pow(goal_point[1] - init_state[1], 2)) < 0.5) {
    //     std::cout << "Goal reached!" << std::endl;
    //     goal_index++;
    // }

    if (sqrt(pow(goal_point[0] - init_state[0], 2) + pow(goal_point[1] - init_state[1], 2)) < 0.5) {
        std::cout << "Goal reached!" << std::endl;
        goal_index++;
    }

    

    if (global_path.size() <= goal_index){
        arrive_flag = true;
        goal_index=0;
    }
}


void publishCostmapAndPath() {

    publishCostmap();

    visualizeStartAndGoal(start, goal); // 시작점과 목표점 시각화


    // 경로가 계산되지 않았다면 계산합니다.
    if (!path_computed) {
        std::vector<Node> path; // A star 경로
        std::vector<Node> expand;

        if (a_star_->plan(start, goal, path, expand)) {
            global_path = path; // 전역 경로에 저장

            for(auto& p:path)
            {
                std::cout << "path: " << p.x() << " "<< p.y() << "\n";
            }
            
            path_computed = true; // 경로가 계산되었음을 표시
            
            std::cout << "Find path" << "\n";
        }
    } 
    else
     {      
        // 경로가 계산되었다면 지속적으로 시각화, ld 계산
        visualizePath(global_path);
        findClosestNode(global_path, goal_index);
        SetDWA();
    }
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("path_planner_node");

    path_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("a_star", 10);
    node_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("node", 10);
    costmap_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
    multi_array_publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("xyflag", 10);
    trajectory_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("local_path", 10);

    auto subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>("pose", 10, topic_callback);
    

    unsigned int cells_size_x = 300;
    unsigned int cells_size_y = 250;
    double resolution = 1;


    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
        cells_size_x, cells_size_y, resolution, origin_x, origin_y);


    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);


    // PCD 파일 경로 설정
    std::string pcd_file_path = "/home/jinho/Downloads/map_2d.pcd";
    loadPointCloudFromPCD(pcd_file_path);


    a_star_ = std::make_unique<global_planner::AStar>(costmap_.get(), false, true);

  
    rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
        std::chrono::milliseconds(500), // 0.5초에 한번씩 계산
        []() {
            publishCostmapAndPath();
            publishStaticTransform();
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
