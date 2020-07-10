#ifndef CAMERA_POSE_HPP__
#define CAMERA_POSE_HPP__

#include <opencv2/opencv.hpp>

class Extractor
{
public:
    // Extractor();

    enum DetectorType
    {
        ORB = 0,
        AKAZE = 1,
        BRISK = 2,
        SIFT = 3,
        SURF = 4,
        BRIEF = 5
    };

    void extractAndcompute(const int &num)
    {
        if (num == DetectorType::ORB)
            extractORB();
        else if (num == DetectorType::AKAZE)
            extractAKAZE();
        else if (num == DetectorType::BRISK)
            extractBRISK();
        else if (num == DetectorType::SIFT)
            extractSIFT();
        else if (num == DetectorType::SURF)
            extractSURF();
        else if (num == DetectorType::BRIEF)
            extractBRIEF();
        else
        {
            printf("Choosing Incorrect Option of Feature point detector.\n");
            return;
        }
        detector_->detectAndCompute(this->image, cv::noArray(), this->keypoints, this->descirptors);
        if (descirptors.type() != CV_32F)
            descirptors.convertTo(descirptors, CV_32F);
    }

    void match2point_query(std::vector<cv::DMatch> matches)
    {
        point.clear();
        for (size_t i = 0; i < matches.size(); i++)
        {
            cv::Point2f p;
            p.x = keypoints[matches[i].queryIdx].pt.x;
            p.y = keypoints[matches[i].queryIdx].pt.y;
            point.push_back(p);
        }
    }

    void match2point_train(std::vector<cv::DMatch> matches)
    {
        point.clear();
        for (size_t i = 0; i < matches.size(); i++)
        {
            cv::Point2f p;
            p.x = keypoints[matches[i].trainIdx].pt.x;
            p.y = keypoints[matches[i].trainIdx].pt.y;
            point.push_back(p);
        }
    }

private:
    void extractORB()
    {
        detector_ = cv::ORB::create(1000, 1.2f, 30, 21, 0, 2, cv::ORB::FAST_SCORE, 31, 5);
    }
    void extractAKAZE()
    {
        detector_ = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0000001f);
    }
    void extractBRISK()
    {
        detector_ = cv::BRISK::create(120, 3, 0.6f);
    }
    void extractSIFT()
    {
        printf("まだSIFTは対応してないよ！\n");
    }
    void extractSURF()
    {
        printf("まだSURFは対応してないよ！\n");
    }
    void extractBRIEF()
    {
        printf("まだBRIEFは対応してないよ！\n");
    }

public:
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descirptors;
    std::vector<cv::Point2f> point;

private:
    cv::Ptr<cv::Feature2D> detector_;
};

class CameraInfo
{
public:
    CameraInfo()
        : CameraMatrix(3, 3, CV_32FC1), CameraPose(3, 4, CV_32FC1), ProjectionMatrix(3, 4, CV_32FC1),
          Rotation(3, 3, CV_32FC1), Rotation_world(3, 3, CV_32FC1),
          Transform(3, 1, CV_32FC1), Transform_world(3, 1, CV_32FC1),
          frame_num(0){};
    void setData()
    {
        cv::Mat CamPose(3, 4, CV_32FC1);
        // 並進についてはカメラ→ワールド座標原点のベクトル（ワールド座標系）
        // 回転についてはカメラ→ワールド座標系の回転行列
        cv::hconcat(Rotation_world.t(), -Rotation_world.t() * Transform_world, CamPose);
        this->CameraPose = CamPose.clone();

        // 射影行列
        cv::Mat ProjMat(3, 4, CV_32FC1);
        ProjMat = CameraMatrix * CameraPose;
        this->ProjectionMatrix = ProjMat.clone();
    }

public:
    cv::Mat CameraMatrix;
    cv::Mat CameraPose;
    cv::Mat ProjectionMatrix;
    cv::Mat Rotation;
    cv::Mat Rotation_world;
    cv::Mat Transform;
    cv::Mat Transform_world;
    int frame_num;
};

class MatchedData
{
public:
    MatchedData();
    MatchedData(const cv::Point2f &point, const cv::Mat &PrjMat,
                const cv::Mat &Rot, const cv::Mat &Trans, const int &Frame_Num)
    {
        image_points = point;
        ProjectionMatrix = PrjMat.clone();
        Rotation_world = Rot.clone();
        Transform_world = Trans.clone();
        frame_num = Frame_Num;
    };
    cv::Point2f image_points;
    cv::Mat ProjectionMatrix;
    cv::Mat Rotation_world;
    cv::Mat Transform_world;
    int frame_num;
};

class FrameDatabase
{
public:
    FrameDatabase()
        : extractor(), camerainfo()
    {
    }
    Extractor extractor;
    CameraInfo camerainfo;
    std::multimap<unsigned int, MatchedData> keyponit_map;
};

#endif