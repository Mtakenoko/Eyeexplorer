#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>

int main(void)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 10;
    cloud.height = 10;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);
    cloud.points[0] = pcl::PointXYZ(0, 0, 0);

    pcl::visualization::CloudViewer viewer("simple cloud viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    viewer.showCloud(cloud_ptr);
    
}
