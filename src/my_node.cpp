#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr map_in(new pcl::PointCloud<pcl::PointXYZ>());

void load_map()
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jinho/Downloads/GlobalMap.pcd", *map_in) == -1)
    {
        return;
    }
}

void roi()
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(map_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.3, 1.0);
    pass.filter(*map_in);
}

void voxel()
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setLeafSize(0.3, 0.3, 0.3);
    sor.setInputCloud(map_in);
    sor.filter(*map_in);
}

void press()
{
    for (auto& point : map_in->points)
    {
        point.x *= 10.0; // x 좌표를 두 배로 확장
        point.y *= 10.0; // y 좌표를 두 배로 확장
        point.z = 0.0;  // z 좌표는 0으로 설정
    }
}

void view()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Layered Cloud Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(map_in->makeShared());
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void save()
{
    pcl::io::savePCDFileASCII("/home/jinho/Downloads/map_2d.pcd", *map_in);
}

int main()
{
    load_map();
    std::cout << "Initial size: " << map_in->size() << std::endl;
    roi();
    voxel(); // voxel 필터링을 먼저 적용
    press(); // 점 간 간격을 늘림
    std::cout << "Final size: " << map_in->size() << std::endl;
    view();
    save();
    return 0;
}
