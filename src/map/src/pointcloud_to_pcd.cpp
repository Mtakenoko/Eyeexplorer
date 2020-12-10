#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <ctime>

#include "../include/map/pointcloud_to_pcd.hpp"

PointCloud_to_PCD::PointCloud_to_PCD() : viewer("cloud viewer"),
                                        //  saved_viewer("Saved cloud Viewer"),
                                         cloud_(new pcl::PointCloud<pcl::PointXYZ>()), saved_cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {}

void PointCloud_to_PCD::topic_callback_(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud)
{
    // SubscribeしたものをPCL用データに変換
    this->cloud_ = PointCloud_to_PCD::input_data(msg_pointcloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_to_PCD::input_data(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_(new pcl::PointCloud<pcl::PointXYZ>());

    input_cloud_->width = (int)msg_pointcloud->markers.size();
    input_cloud_->height = 1;
    input_cloud_->is_dense = false;
    // input_cloud_->resize(input_cloud_->width * input_cloud_->height);
    for (auto itr = msg_pointcloud->markers.begin(); itr != msg_pointcloud->markers.end(); itr++)
    {
        pcl::PointXYZ xyz;
        xyz.x = itr->pose.position.x;
        xyz.y = itr->pose.position.y;
        xyz.z = itr->pose.position.z;
        input_cloud_->points.push_back(xyz);
    }
    return input_cloud_;
}

void PointCloud_to_PCD::save(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_)
{
    if (this->cloud_->size() != 0)
    {
        time_t rawtime;
        struct tm *timeinfo;
        char buffer[80];

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
        std::string str(buffer);

        std::cout << "saving PCD File 'ASCII'" << std::endl;
        pcl::io::savePCDFileASCII("/home/takeyama/workspace/ros2_eyeexplorer/src/map/output/" + str + "_cloud_ascii.pcd", *input_cloud_);
        std::cout << "saving PCD File 'Binary'" << std::endl;
        pcl::io::savePCDFileASCII("/home/takeyama/workspace/ros2_eyeexplorer/src/map/output/" + str + "_cloud_binary.pcd", *input_cloud_);

        this->saved_cloud_ = input_cloud_;
    }
}

void PointCloud_to_PCD::showPointClooud()
{
    if (!cloud_->empty())
        viewer.showCloud(this->cloud_);
}

void PointCloud_to_PCD::showSavedPCD()
{
    // if (!saved_cloud_->empty())
    //     saved_viewer.showCloud(this->saved_cloud_);
}

void PointCloud_to_PCD::savePCD()
{
    if (!cloud_->empty())
        PointCloud_to_PCD::save(this->cloud_);
}

int PointCloud_to_PCD::getPointCloudNum()
{
    return (int)this->cloud_->size();
}