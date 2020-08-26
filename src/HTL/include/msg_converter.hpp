#ifndef MSG_CONVERTER__
#define MSG_CONVERTER__

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

class Converter
{
public:
    static void cvimage_to_msg(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg);
    static void cvMat_to_msgPointCloud(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, const size_t &color);
    static void cvMat_to_msgPointCloud2(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, int dist_count = 0);
    static std::string mat_type2encoding(int mat_type);
    static pcl::PointXYZRGB pclColor(const size_t &color);

    enum Color
    {
        WHITE = 0,
        BRACK = 1,
        SILVER = 2,
        BLUE = 3,
        RED = 4,
        GREEN = 5,
        YELLOW = 6,
        AQUA = 7,
        PINK = 8,
        OLIVE = 9,
        TEAL = 10,
        SKIN = 11
    };
};

#endif