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
#include <cstddef> // size_t를 사용하기 위해 추가

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr node_publisher_;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;

std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
std::unique_ptr<global_planner::AStar> a_star_;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

// 경로를 저장할 변수 추가
std::vector<Node> global_path;
bool path_computed = false; // 경로가 계산되었는지 여부를 확인하는 플래그

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
    start_marker.scale.x = 0.2; // 크기
    start_marker.scale.y = 0.2; // 크기
    start_marker.scale.z = 0.2; // 크기
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
    goal_marker.scale.x = 0.2; // 크기
    goal_marker.scale.y = 0.2; // 크기
    goal_marker.scale.z = 0.2; // 크기
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
        if (costmap_->worldToMap(point.x, point.y, mx, my)) {
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
}

void publishCostmap() {
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.header.stamp = rclcpp::Clock().now();
    occupancy_grid.info.resolution = costmap_->getResolution();

    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();
    occupancy_grid.info.width = size_x;
    occupancy_grid.info.height = size_y;

    occupancy_grid.data.resize(size_x * size_y);

    for (unsigned int i = 0; i < size_x; ++i) {
        for (unsigned int j = 0; j < size_y; ++j) {
            unsigned char cost = costmap_->getCost(i, j);
            occupancy_grid.data[j * size_x + i] = cost;
        }
    }

    costmap_publisher_->publish(occupancy_grid);
}

void publishCostmapAndPath() {
    publishCostmap();

    Node start(0, 0);
    Node goal(30, 50);

    visualizeStartAndGoal(start, goal); // 시작점과 목표점 시각화
    

    // 경로가 계산되지 않았다면 계산합니다.
    if (!path_computed) {

        std::vector<Node> path;
        std::vector<Node> expand;

        if (a_star_->plan(start, goal, path, expand)) {
            global_path = path; // 전역 경로에 저장

            for(auto& p:path)
            {
                std::cout << "path: " << p.x() << " "<< p.y() << "\n";
            }
            
            path_computed = true; // 경로가 계산되었음을 표시
        }
    } 
    else
     {
        // 경로가 계산되었다면 지속적으로 시각화합니다.
        visualizePath(global_path);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("path_planner_node");

    path_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("a_star", 10);
    node_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("node", 10);
    costmap_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
    
    unsigned int cells_size_x = 100;
    unsigned int cells_size_y = 100;
    double resolution = 0.5;
    double origin_x = -34;
    double origin_y = -30;

    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
        cells_size_x, cells_size_y, resolution, origin_x, origin_y);

    a_star_ = std::make_unique<global_planner::AStar>(costmap_.get(), false, true);

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    // PCD 파일 경로 설정 (파일 경로를 적절히 수정하세요)
    std::string pcd_file_path = "/home/jinho/Downloads/map_2d.pcd";
    loadPointCloudFromPCD(pcd_file_path);

    rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        []() {
            publishCostmapAndPath();
            publishStaticTransform();
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
