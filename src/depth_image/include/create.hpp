#ifndef DEPTH_IMAGE_CREATE_HPP_
#define DEPTH_IMAGE_CREATE_HPP_

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>

class Depth_Create
{
public:
    Depth_Create();
    void topic_callback_image_(const sensor_msgs::msg::Image::SharedPtr msg_image);
    void topic_callback_transform_(const geometry_msgs::msg::Transform::SharedPtr msg_transform);
    void topic_callback_model_(const visualization_msgs::msg::Marker::SharedPtr msg_model);
    int getSceneNum();
    int getUseSceneNum();
    void getNowImage(cv::Mat *image);
    void getNewSceneImage(cv::Mat *image);
    void getNewMarkerImage(cv::Mat *image);
    bool getFinishFlag();
    void setCaptureFlag();
    void resetScene();
    void deleteScene();
    void saveScene();

private:
    void input_image_data(const sensor_msgs::msg::Image::SharedPtr msg_image);
    void input_transform_data(const geometry_msgs::msg::Transform::SharedPtr msg_image);
    void input_model_data(const visualization_msgs::msg::Marker::SharedPtr msg_image);
    void clear();
    int encoding2mat_type(const std::string &encoding);
    void setNewScene();

private:
    cv::Mat now_image;
    cv::Mat scene_image;
    cv::Mat marker_image;

private:
    double *offset_output;

public:
    int scene_counter;
    int use_scene_counter;

private:
    bool flag_set;
    bool flag_set_image;
    bool flag_set_transform;
    bool flag_set_model;
    bool flag_finish;
};
#endif