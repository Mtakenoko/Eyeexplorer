#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include "../include/create.hpp"
#include "../../htl/include/transform.hpp"

Depth_Create::Depth_Create()
    : scene_counter(0), flag_set(false)
{
    std::cout << "Welcome!" << std::endl;
}

void Depth_Create::topic_callback_image_(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    this->input_image_data(msg_image);
    if (flag_set && scene.flag_set_image && scene.flag_set_model && scene.flag_set_transform)
        this->calc();
}

void Depth_Create::topic_callback_transform_(const geometry_msgs::msg::Transform::SharedPtr msg_transform)
{
    this->input_transform_data(msg_transform);
}

void Depth_Create::topic_callback_model_(const visualization_msgs::msg::Marker::SharedPtr msg_model)
{
    this->input_model_data(msg_model);
    if (flag_set && scene.flag_set_image && scene.flag_set_model && scene.flag_set_transform)
        this->calc();
}

int Depth_Create::encoding2mat_type(const std::string &encoding)
{
    if (encoding == "mono8")
        return CV_8UC1;
    else if (encoding == "bgr8")
        return CV_8UC3;
    else if (encoding == "mono16")
        return CV_16SC1;
    else if (encoding == "rgba8")
        return CV_8UC4;
    else if (encoding == "bgra8")
        return CV_8UC4;
    else if (encoding == "32FC1")
        return CV_32FC1;
    else if (encoding == "rgb8")
        return CV_8UC3;
    else
        throw std::runtime_error("Unsupported encoding type");
}

void Depth_Create::input_image_data(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    // 画像
    cv::Mat frame_image(msg_image->height, msg_image->width, Depth_Create::encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    now_image = frame_image.clone();
    scene.flag_set_image = true;
    // printf("sub[image] : #%s\n", msg_image->header.frame_id.c_str());
}

void Depth_Create::input_transform_data(const geometry_msgs::msg::Transform::SharedPtr msg_transform)
{
    scene.Rotation_world = htl::Transform::QuaternionToRotMat<double>(msg_transform->rotation.x, msg_transform->rotation.y, msg_transform->rotation.z, msg_transform->rotation.w);
    scene.Transform_world = (cv::Mat_<double>(3, 1) << msg_transform->translation.x, msg_transform->translation.y, msg_transform->translation.z);
    scene.flag_set_transform = true;
    // printf("sub[trans] : [%0.3lf %0.3lf %0.3lf]\n", msg_transform->translation.x, msg_transform->translation.y, msg_transform->translation.z);
}

void Depth_Create::input_model_data(const visualization_msgs::msg::Marker::SharedPtr msg_model)
{
    scene.eyemodel = *msg_model;
    scene.flag_set_model = true;
    // printf("sub[model] : [%0.3lf %0.3lf %0.3lf]\n", msg_model->pose.position.x, msg_model->pose.position.y, msg_model->pose.position.z);
}

void Depth_Create::calc()
{
    scene.color_image = now_image.clone();
    scene_counter++;
    flag_set = false;
}

void Depth_Create::setCaptureFlag()
{
    flag_set = true;
}

int Depth_Create::getSceneNum()
{
    return scene_counter;
}

void Depth_Create::getNowImage(cv::Mat *image)
{
    if (!now_image.empty())
        *image = now_image.clone();
}

void Depth_Create::getNewSceneImage(cv::Mat *image)
{
    if (!scene.color_image.empty())
        *image = scene.color_image.clone();
}

void Depth_Create::getNewDepthImage(cv::Mat *image)
{
    if (!scene.depth_image.empty())
        *image = scene.depth_image.clone();
}

void Depth_Create::clear()
{
    scene_counter = 0;
    flag_set = false;
    std::cout << "scene was clear" << std::endl;
}

void Depth_Create::deleteScene()
{
    flag_set = false;
    std::cout << "scene was deleted" << std::endl;
}
void Depth_Create::saveScene()
{
    flag_set = false;
    std::cout << "scene was saved" << std::endl;
}