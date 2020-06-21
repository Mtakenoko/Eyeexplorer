#include <opencv2/opencv.hpp>
#include "../include/calibration_delay/Scene.hpp"

void Marker::setPosition(int marker_id)
{
    // マーカーIDを受け取ればそれに対応する三次元位置をPositionに格納する
    switch (marker_id)
    {
    case 0:
        this->Position.x = -234.5;
        this->Position.y = -368.5;
        this->Position.z = -56.0;
        break;
    case 1:
        this->Position.x = -234.5;
        this->Position.y = -306.0;
        this->Position.z = -56.0;
        break;
    case 2:
        this->Position.x = -234.5;
        this->Position.y = -243.0;
        this->Position.z = -56.0;
        break;
    case 3:
        this->Position.x = -234.5;
        this->Position.y = -180.3;
        this->Position.z = -56.0;
        break;
    case 4:
        this->Position.x = -234.5;
        this->Position.y = -306.0;
        this->Position.z = -108.2;
        break;
    case 5:
        this->Position.x = -234.5;
        this->Position.y = -243.0;
        this->Position.z = -108.2;
        break;
    case 6:
        this->Position.x = -234.5;
        this->Position.y = -368.5;
        this->Position.z = -161.0;
        break;
    case 7:
        this->Position.x = -234.5;
        this->Position.y = -306.0;
        this->Position.z = -161.0;
        break;
    case 8:
        this->Position.x = -234.5;
        this->Position.y = -243.0;
        this->Position.z = -161.0;
        break;
    case 9:
        this->Position.x = -234.5;
        this->Position.y = -180.3;
        this->Position.z = -161.0;
        break;
    case 10:
        this->Position.x = -192.0;
        this->Position.y = 341.5;
        this->Position.z = -210.0;
        break;
    case 11:
        this->Position.x = -192.0;
        this->Position.y = 404.0;
        this->Position.z = -210.0;
        break;
    case 12:
        this->Position.x = -192.0;
        this->Position.y = 467.0;
        this->Position.z = -210.0;
        break;
    case 13:
        this->Position.x = -192.0;
        this->Position.y = 529.7;
        this->Position.z = -210.0;
        break;
    case 14:
        this->Position.x = -139.8;
        this->Position.y = 404.0;
        this->Position.z = -210.0;
        break;
    case 15:
        this->Position.x = -139.8;
        this->Position.y = 467.0;
        this->Position.z = -210.0;
        break;
    case 16:
        this->Position.x = -87.0;
        this->Position.y = 341.5;
        this->Position.z = -210.0;
        break;
    case 17:
        this->Position.x = -87.0;
        this->Position.y = 404.0;
        this->Position.z = -210.0;
        break;
    case 18:
        this->Position.x = -87.0;
        this->Position.y = 467.0;
        this->Position.z = -210.0;
        break;
    case 19:
        this->Position.x = -87.0;
        this->Position.y = 529.7;
        this->Position.z = -210.0;
        break;
    default:
        std::cerr << "marker_id is unsupported" << std::endl;
        break;
    }
}

Scene::Scene()
    : joint_counter(0),
      set_joint_flag(true), set_image_flag(true), finish_flag(false)
{
}