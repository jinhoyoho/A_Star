#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr map_in(new pcl::PointCloud<pcl::PointXYZ>());

void load_map()
{
    // PCD 파일을 로드
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/leesh/Downloads/service_LOAM/GlobalMap.pcd", *map_in) == -1)
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/leesh/Downloads/GlobalMap.pcd", *map_in) == -1)
    {
        return ;
    }
}

void roi()
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(map_in);

    // Z 축 기준으로 필터링 (예: Z축 범위 0.0 ~ 1.0)
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.3, 1.0);
    pass.filter(*map_in);
}

void press()
{
    for (auto& point : map_in->points)
    {
        point.z = 0.0;
    }
}

void voxel()
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setLeafSize(0.3, 0.3, 0.3);
    sor.setInputCloud(map_in);
    sor.filter(*map_in);
}

void view()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Layered Cloud Viewer"));

    viewer->addPointCloud<pcl::PointXYZ>(map_in->makeShared());


    // Viewer가 닫힐 때까지 기다림
    viewer->setBackgroundColor(0, 0, 0); // 배경 색을 검은색으로 설정
    viewer->addCoordinateSystem(1.0); // 좌표계를 추가
    viewer->initCameraParameters(); // 카메라 파라미터 초기화

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void save()
{
    pcl::io::savePCDFileASCII("/home/leesh/Downloads/service_LOAM/map_2d.pcd", *map_in);
}

int main(int argc, char *argv[])
{
    load_map();
    std::cout << map_in->size() << std::endl;
    roi();
    press();
    voxel();
    std::cout << map_in->size() << std::endl;
    view();
    save();
    return 0;
}