#ifndef MSG_CONVERTER__
#define MSG_CONVERTER__

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/point32.hpp>

class Converter
{
public:
    explicit Converter();
    static void cvimage_to_msg(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg);
    static void cvMat_to_msgPointCloud(const cv::Mat pointCloud, sensor_msgs::msg::PointCloud &msg_cloud_pub);
    static void cvMat_to_msgPointCloud2(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, int dist_count = 0);
    static std::string mat_type2encoding(int mat_type);
private:

};


#endif