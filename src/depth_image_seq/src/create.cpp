#include <fstream>
#include <time.h>
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>
#include "../include/create.hpp"
#include "../include/depth.hpp"
#include "/home/takeyama/workspace/htl/opencv/transform.hpp"

Depth_Create::Depth_Create()
    : scene_counter(0), flag_set(false), flag_set_model(false), flag_mkdir(true) {}

void Depth_Create::topic_callback_image_(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    int frame_now = std::stoi(msg_image->header.frame_id);
    static int pre_frame = 0;
    int frame_span = frame_now - pre_frame;
    // std::cout << "frame_num : " << frame_now << ", pre_frame : " << pre_frame << std::endl;
    if (frame_span < NUM_SKIP_FRAMES)
        return;
    this->input_image_data(msg_image);
    // std::cout << "flag_set : " << flag_set << std::endl;
    // std::cout << "scene.flag_set_image : " << scene.flag_set_image << std::endl;
    // std::cout << "scene.flag_set_transform : " << scene.flag_set_transform << std::endl;
    // std::cout << "flag_set_model : " << flag_set_model << std::endl;
    if (flag_set && scene.flag_set_image && scene.flag_set_transform && flag_set_model)
    {
        this->scene_container.push_back(scene);
        std::cout << "scene_container.size() : " << scene_container.size() << std::endl;
        scene.flag_set_image = false;
        scene.flag_set_transform = false;

        if (scene_container.size() == NUM_FRAMES)
            this->calc();
    }
    pre_frame = frame_now;
}

void Depth_Create::topic_callback_transform_(const geometry_msgs::msg::Transform::SharedPtr msg_transform)
{
    this->input_transform_data(msg_transform);
}

void Depth_Create::topic_callback_model_(const visualization_msgs::msg::Marker::SharedPtr msg_model)
{
    this->input_model_data(msg_model);
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
    scene.color_image = frame_image.clone();
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
    eyemodel = *msg_model;
    flag_set_model = true;
}

void Depth_Create::calc()
{
    for (auto itr = scene_container.begin(); itr != scene_container.end(); itr++)
    {
        // まずモデル内部に内視鏡が存在しているかチェックする
        if (!itr->isInModel(eyemodel))
        {
            std::cout << "Endoscope is not in the eyeball model" << std::endl;
            this->deleteScene();
            return;
        }

        // 深度画像計算
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fovx, 0.0, u0,
                                0.0, fovy, v0,
                                0.0, 0.0, 1.0);
        DepthModel<double> depthmodel;
        depthmodel.setImageInfo(itr->color_image.rows, itr->color_image.cols, cameraMatrix);
        depthmodel.setCameraPose(itr->Rotation_world, itr->Transform_world);
        depthmodel.setModel(eyemodel);
        depthmodel.setMaxDistance(MAX_DIST);
        itr->depth_image = depthmodel.create();

        // フラグ管理
        if (!itr->depth_image.empty())
            itr->flag_caliculated = true;
        else
            itr->flag_caliculated = false;
        itr->flag_set_image = false;
        itr->flag_set_transform = false;
    }
    std::cout << "#" << scene_counter << " scene was caliculated" << std::endl;
    flag_set = false;
}

void Depth_Create::setCaptureFlag()
{
    std::cout << std::endl;
    std::cout << "Start capturing" << std::endl;
    flag_set = true;
    scene_container.clear();
    scene_counter++;
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
    scene_container.clear();
    flag_set = false;
    scene.flag_set_image = false;
    scene.flag_set_transform = false;
    scene.flag_caliculated = false;
    std::cout << "scene was deleted" << std::endl;
}

void Depth_Create::saveScene()
{
    if (scene_container.size() != NUM_FRAMES)
    {
        std::cout << "scene_container.size() != NUM_FRAMES" << std::endl;
        return;
    }

    for (auto itr = scene_container.begin(); itr != scene_container.end(); itr++)
    {
        if (!itr->flag_caliculated)
        {
            std::cout << "There is a scene which can't caliculate depth" << std::endl;
            return;
        }
    }

    // mkdir
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    if (flag_mkdir)
    {
        filepath_1 = "/home/takeyama/workspace/ros2_eyeexplorer/src/depth_image_seq/Output/" +
                     std::to_string(pnow->tm_year + 1900) +
                     std::to_string(pnow->tm_mon + 1) +
                     std::to_string(pnow->tm_mday + 1) +
                     std::to_string(pnow->tm_hour + 1) +
                     std::to_string(pnow->tm_min + 1) +
                     std::to_string(pnow->tm_sec) + "/";
        struct stat st1;
        if (stat(filepath_1.c_str(), &st1) != 0)
        {
            std::cout << "mkdir : " << filepath_1 << std::endl;
            mkdir(filepath_1.c_str(), 0775);
        }
        flag_mkdir = false;
    }
    filepath_2 = std::to_string(scene_counter) + "/";
    struct stat st2;
    if (stat((filepath_1 + filepath_2).c_str(), &st2) != 0)
    {
        mkdir((filepath_1 + filepath_2).c_str(), 0775);
    }

    // Save
    for (auto itr = scene_container.begin(); itr != scene_container.end(); itr++)
    {
        if (!itr->color_image.empty() && !itr->depth_image.empty())
        {
            cv::imwrite(filepath_1 + filepath_2 + std::to_string(std::distance(scene_container.begin(), itr)) + ".jpg", itr->color_image);
            cv::imwrite(filepath_1 + filepath_2 + std::to_string(std::distance(scene_container.begin(), itr)) + ".png", itr->depth_image);
            scene.flag_caliculated = false;
        }
    }
    std::cout << "#" << scene_counter << " scene was saved" << std::endl;
    flag_set = false;
}