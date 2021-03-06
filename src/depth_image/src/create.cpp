#include <fstream>
#include <time.h>
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>
#include "../include/create.hpp"
#include "../include/depth.hpp"
#include "/home/takeyama/workspace/htl/opencv/transform.hpp"

Depth_Create::Depth_Create()
    : scene_counter(0), flag_set(false), flag_mkdir(true) {}

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
}

void Depth_Create::input_transform_data(const geometry_msgs::msg::Transform::SharedPtr msg_transform)
{
    scene.Rotation_world = htl::Transform::QuaternionToRotMat<double>(msg_transform->rotation.x, msg_transform->rotation.y, msg_transform->rotation.z, msg_transform->rotation.w);
    scene.Transform_world = (cv::Mat_<double>(3, 1) << msg_transform->translation.x, msg_transform->translation.y, msg_transform->translation.z);
    scene.flag_set_transform = true;
}

void Depth_Create::input_model_data(const visualization_msgs::msg::Marker::SharedPtr msg_model)
{
    scene.eyemodel = *msg_model;
    scene.flag_set_model = true;
}

void Depth_Create::calc()
{
    scene.color_image = now_image.clone();

    // まずモデル内部に内視鏡が存在しているかチェックする
    if (!scene.isInModel())
    {
        std::cout << "Endoscope is not in the eyeball model" << std::endl;
        scene.flag_caliculated = false;
        scene.flag_set_image = false;
        scene.flag_set_transform = false;
        scene.flag_set_model = false;
        flag_set = false;
        return;
    }

    // 深度画像計算
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fovx, 0.0, u0,
                            0.0, fovy, v0,
                            0.0, 0.0, 1.0);
    DepthModel<double> depthmodel;
    depthmodel.setImageInfo(now_image.rows, now_image.cols, cameraMatrix);
    depthmodel.setCameraPose(scene.Rotation_world, scene.Transform_world);
    depthmodel.setModel(scene.eyemodel);
    depthmodel.setMaxDistance(MAX_DIST);
    scene.depth_image = depthmodel.create();

    // フラグ管理
    if (!scene.depth_image.empty())
        scene.flag_caliculated = true;
    else
        scene.flag_caliculated = false;
    scene.flag_set_image = false;
    scene.flag_set_transform = false;
    scene.flag_set_model = false;
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
        cv::cvtColor(scene.depth_image, *image, cv::COLOR_GRAY2BGR);
}

void Depth_Create::deleteScene()
{
    flag_set = false;
    scene.flag_set_image = false;
    scene.flag_set_transform = false;
    scene.flag_set_model = false;
    std::cout << "scene was deleted" << std::endl;
}

void Depth_Create::saveScene()
{
    if (!scene.flag_caliculated)
        return;

    if (flag_mkdir)
    {
        time_t now = time(NULL);
        struct tm *pnow = localtime(&now);
        filepath = "/home/takeyama/workspace/ros2_eyeexplorer/src/depth_image/Output/" +
                   std::to_string(pnow->tm_year + 1900) +
                   std::to_string(pnow->tm_mon + 1) +
                   std::to_string(pnow->tm_mday + 1) +
                   std::to_string(pnow->tm_hour + 1) +
                   std::to_string(pnow->tm_min + 1) +
                   std::to_string(pnow->tm_sec) + "/";
        struct stat st;
        if (stat(filepath.c_str(), &st) != 0)
        {
            std::cout << "mkdir : " << filepath << std::endl;
            mkdir(filepath.c_str(), 0775);
        }
        flag_mkdir = false;
    }

    if (!scene.color_image.empty() && !scene.depth_image.empty())
    {
        cv::imwrite(filepath + std::to_string(scene_counter) + ".jpg", scene.color_image);
        cv::imwrite(filepath + std::to_string(scene_counter) + ".png", scene.depth_image);
        std::cout << "#" << scene_counter << " scene was saved" << std::endl;
        scene.flag_caliculated = false;
        scene_counter++;
    }
    flag_set = false;
}