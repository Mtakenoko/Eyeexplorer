#ifndef DEPTH_IMAGE_CREATE_HPP_
#define DEPTH_IMAGE_CREATE_HPP_

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>

#define fovx 396.7
#define fovy 396.9
#define u0 163.6
#define v0 157.1
#define MAX_DIST 0.025

class Scene
{
public:
    Scene() : flag_set_image(false), flag_set_transform(false), flag_set_model(false),
              flag_caliculated(false) {}
    bool isInModel()
    {
        double distance = (eyemodel.scale.x + eyemodel.scale.y + eyemodel.scale.z) / 6. -
                          std::sqrt((Transform_world.at<double>(0) - eyemodel.pose.position.x) * (Transform_world.at<double>(0) - eyemodel.pose.position.x) +
                                    (Transform_world.at<double>(1) - eyemodel.pose.position.y) * (Transform_world.at<double>(1) - eyemodel.pose.position.y) +
                                    (Transform_world.at<double>(2) - eyemodel.pose.position.z) * (Transform_world.at<double>(2) - eyemodel.pose.position.z));
        return distance > 0;
    }

public:
    cv::Mat color_image;
    cv::Mat depth_image;
    size_t frame_num;
    cv::Mat Rotation_world;
    cv::Mat Transform_world;
    visualization_msgs::msg::Marker eyemodel;
    bool flag_set_image;
    bool flag_set_transform;
    bool flag_set_model;
    bool flag_caliculated;
};

class Depth_Create
{
public:
    Depth_Create();
    void topic_callback_image_(const sensor_msgs::msg::Image::SharedPtr msg_image);
    void topic_callback_transform_(const geometry_msgs::msg::Transform::SharedPtr msg_transform);
    void topic_callback_model_(const visualization_msgs::msg::Marker::SharedPtr msg_model);
    int getSceneNum();
    void getNowImage(cv::Mat *image);
    void getNewSceneImage(cv::Mat *image);
    void getNewDepthImage(cv::Mat *image);
    void setCaptureFlag();
    void deleteScene();
    void saveScene();

private:
    void input_image_data(const sensor_msgs::msg::Image::SharedPtr msg_image);
    void input_transform_data(const geometry_msgs::msg::Transform::SharedPtr msg_image);
    void input_model_data(const visualization_msgs::msg::Marker::SharedPtr msg_image);
    void calc();
    void clear();
    int encoding2mat_type(const std::string &encoding);
    void setNewScene();

private:
    cv::Mat now_image;
    Scene scene;

private:
    double *offset_output;
    std::string filepath;

public:
    int scene_counter;

private:
    bool flag_set;
    bool flag_mkdir;
};
#endif