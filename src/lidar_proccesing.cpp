#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor() : Node("point_cloud_processor")
    {
        // Subscriber to /velodyne_points
        velodyne_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10, std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

        // Subscriber to /localization
        localization_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/pose", 10, std::bind(&PointCloudProcessor::localizationCallback, this, std::placeholders::_1));

        // Publisher for processed point cloud
        processed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_points", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Filter points with Z <= -0.3 and project to XY plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& point : cloud->points)
        {
            if (point.z > -0.3)
            {
                pcl::PointXYZ new_point = point;
                new_point.z = 0.0; // Project to XY plane
                filtered_cloud->points.push_back(new_point);
            }
        }

        // Apply VoxelGrid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        sor.filter(*voxel_filtered_cloud);

        // Transform point cloud to global frame using localization data
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << global_x_, global_y_, 0.0;
        transform.rotate(Eigen::AngleAxisf(global_yaw_, Eigen::Vector3f::UnitZ()));

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*voxel_filtered_cloud, *transformed_cloud, transform);

        // Convert back to ROS PointCloud2 message and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*transformed_cloud, output_msg);
        output_msg.header = msg->header;
        processed_pub_->publish(output_msg);
    }

    void localizationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 3)
        {
            global_x_ = static_cast<float>(msg->data[0]);
            global_y_ = static_cast<float>(msg->data[1]);
            global_yaw_ = static_cast<float>(msg->data[2]); // Convert yaw from degrees to radians
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr localization_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_pub_;

    float global_x_ = 0.0;
    float global_y_ = 0.0;
    float global_yaw_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}

