#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <chrono>
#include <std_msgs/msg/float64_multi_array.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr map_in(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode()
        : Node("localization_node"),
          last_transformation_(Eigen::Matrix4f::Identity())  // Initialize with identity matrix
    {
        load_map();
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&LocalizationNode::pointCloudCallback, this, std::placeholders::_1));
        
        map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_in", 10);
        aligned_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_cloud", 10);
        arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        pose_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("pose", 10);
    }

private:
    void load_map()
    {
        // PCD 파일을 로드
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/leesh/Downloads/service_LOAM/map_2d.pcd", *map_in) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file GlobalMap.pcd");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Loaded GlobalMap.pcd with %d points", map_in->points.size());
    }

    void do_ndt()
    {
        
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
        approximate_voxel_filter.setInputCloud(cloud);
        approximate_voxel_filter.filter(*filtered_cloud);

        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setStepSize(0.2);
        ndt.setResolution(1.0);
        ndt.setMaximumIterations(100);
        ndt.setInputSource(filtered_cloud);
        ndt.setInputTarget(map_in);

        // Use the previous transformation matrix as the initial guess
        Eigen::Matrix4f init_guess = last_transformation_;
        // std::cout << init_guess << std::endl;
        // Calculating required rigid transform to align the input cloud to the target cloud.
        
        ndt.align(*output_cloud, init_guess);

        // std::cout << "Normal Distributions Transform has " << (ndt.hasConverged() ? "converged" : "not converged")
        //           << ", score: " << ndt.getFitnessScore() << std::endl;

        // std::cout << ndt.getFinalTransformation() << std::endl;

        // Save the current transformation matrix for the next iteration
        last_transformation_ = ndt.getFinalTransformation();
    }

    void do_icp()
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance(5);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        //use the outcome of ndt as the initial guess for ICP
        icp.setInputSource(filtered_cloud);
        icp.setInputTarget(map_in);

        Eigen::Matrix4f init_guess = last_transformation_;

        // std::cout << init_guess << std::endl;
        icp.align(*output_cloud, init_guess);

        // std::cout << "Itertate closest point has " << (icp.hasConverged() ? "converged" : "not converged")
        //           << ", score: " << icp.getFitnessScore() << std::endl;

        // std::cout << icp.getFinalTransformation() << std::endl;

        // Save the current transformation matrix for the next iteration
        last_transformation_ = icp.getFinalTransformation();
    }


    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        auto start = std::chrono::high_resolution_clock::now();
        // sensor_msgs::msg::PointCloud2 메시지를 pcl::PointCloud<pcl::PointXYZ>로 변환
        pcl::fromROSMsg(*msg, *cloud);

        // 수신된 포인트 클라우드 처리
        // RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", cloud->points.size());

        // NaN 값 제거
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        
        make_2d();

        do_ndt();

        do_icp();

        publish();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        // 실행 시간을 밀리초 단위로 변환하여 출력
        RCLCPP_INFO(this->get_logger(), "PointCloudCallback execution time: %ld ms", duration / 1000);
    }

    void publish()
    {   
        auto x = last_transformation_(0, 3);
        auto y = last_transformation_(1, 3);

        // yaw는 atan2 함수를 이용해 계산합니다.
        auto yaw = std::atan2(last_transformation_(1, 0), last_transformation_(0, 0));
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, heading: %f", x, y, yaw);

        // 화살표 마커 생성 및 퍼블리시
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp = this->get_clock()->now();
        arrow_marker.ns = "arrow";
        arrow_marker.id = 0;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        // 화살표의 위치 설정
        arrow_marker.pose.position.x = x;
        arrow_marker.pose.position.y = y;
        arrow_marker.pose.position.z = 0.0;

        // 회전을 나타내는 쿼터니언 설정
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw); // yaw만 반영합니다.
        arrow_marker.pose.orientation.x = q.x();
        arrow_marker.pose.orientation.y = q.y();
        arrow_marker.pose.orientation.z = q.z();
        arrow_marker.pose.orientation.w = q.w();

        // 화살표 크기 설정
        arrow_marker.scale.x = 1.0;  // 화살표 길이
        arrow_marker.scale.y = 0.3;  // 화살표 너비
        arrow_marker.scale.z = 0.1;  // 화살표 높이

        // 화살표 색상 설정
        arrow_marker.color.r = 0.0f;
        arrow_marker.color.g = 1.0f;
        arrow_marker.color.b = 0.0f;
        arrow_marker.color.a = 1.0;

        // 퍼블리시
        arrow_publisher_->publish(arrow_marker);

        // [x, y, yaw] 데이터를 퍼블리시하기 위한 Float64MultiArray 메시지 생성
        std_msgs::msg::Float64MultiArray pose_msg;
        pose_msg.data.push_back(x);
        pose_msg.data.push_back(y);
        pose_msg.data.push_back(yaw);

        // 퍼블리시
        pose_publisher_->publish(pose_msg);

        // Publish the aligned point cloud
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        output_msg.header.frame_id = "map";
        aligned_cloud_publisher_->publish(output_msg);

        // Publish the loaded map
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_in, map_msg);
        map_msg.header.frame_id = "map";
        map_publisher_->publish(map_msg);
    }


    void roi()
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);

        // Z 축 기준으로 필터링 (예: Z축 범위 0.0 ~ 1.0)
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.3, 1.0);
        pass.filter(*cloud);

        // pass.setFilterFieldName("y");
        // pass.setFilterLimits(-10, 10);
        // pass.filter(*cloud);

        // pass.setFilterFieldName("x");
        // pass.setFilterLimits(-10, 10);
        // pass.filter(*cloud);
    }

    void press()
    {
        for (auto& point : cloud->points)
        {
            point.z = 0.0;
        }
    }

    void voxel()
    {
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setLeafSize(0.3, 0.3, 0.3);
        sor.setInputCloud(cloud);
        sor.filter(*cloud);
    }

    void make_2d()
    {
        roi();
        press();
        voxel();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pose_publisher_;
    Eigen::Matrix4f last_transformation_;  // Store the last transformation matrix
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
