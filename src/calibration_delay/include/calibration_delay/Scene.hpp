#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <opencv2/opencv.hpp>

#define MAX_DELAY_TIME 1000 //[ms]

struct Marker
{
    cv::Point3f Position;
    cv::Point2f Point_Image;
    int ID;
    void setPosition(int marker_id);
};

struct Joint
{
    double theta_0;
    double theta_1;
    double theta_2;
    double theta_3;
    double theta_4;
};

class Scene
{
public:
    Scene();

public:
    std::vector<Marker> marker;
    Joint joint[MAX_DELAY_TIME];
    int joint_counter;
    cv::Mat Image;
    bool set_joint_flag;
    bool set_image_flag;
    bool finish_flag;
};
#endif