#ifndef EYE_MAP_KEYFRAME__
#define EYE_MAP_KEYFRAME__

#include <opencv2/opencv.hpp>

namespace eye_map
{
class Keyframe
{
public:
    Keyframe(int id, int width, int height, const cv::Mat R, const cv::Mat t, double timestamp, const unsigned char *image, cv::Mat keypoints, std::vector<cv::KeyPoint> descriptor);
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoint;
    cv::Mat descriptor;
    cv::Mat R;
    cv::Mat t;

private:
};
class KeyframeBuffer
{
public:
    KeyframeBuffer();
    void push_back(int id, int width, int height, const cv::Mat R, const cv::Mat t, double timestamp, const unsigned char *image, cv::Mat keypoints, std::vector<cv::KeyPoint> descriptor);
    void releaseBuffer(int id);
    void readBuffer(int id);
    
};
}; // namespace eye_map
#endif