#ifndef TRIANGULATE_HPP__
#define TRIANGULATE_HPP__

#include <opencv2/opencv.hpp>
#define ITR_TRIANGULATE 10
#define TRI_ITERATIVE_TERM 0.001
#define DISTANCE_CENTER 0.003

class Triangulate
{
    struct ImagePair
    {
        ImagePair(int ID1, int ID2)
        {
            first_image_ID = ID1;
            second_image_ID = ID2;
        }

        int first_image_ID;
        int second_image_ID;
    };

public:
    // 多視点での三角測量
    static cv::Mat triangulation(const std::vector<cv::Point2f> &pnt,
                                 const std::vector<cv::Mat> &ProjectionMatrix);
    static cv::Mat triangulation_RANSAC(const std::vector<cv::Point2f> &pnt,
                                        const std::vector<cv::Mat> &ProjectionMatrix);
    static cv::Mat triangulation_RANSAC(const std::vector<cv::Point2f> &pnt,
                                        const std::vector<cv::Mat> &ProjectionMatrix,
                                        std::vector<bool> &eliminated_scene,
                                        const size_t min_reconstruction_scene);
    // 2視点での三角測量
    static cv::Mat triangulation(const cv::Point2f &pnt1, const cv::Mat &PrjMat1,
                                 const cv::Point2f &pnt2, const cv::Mat &PrjMat2);

private:
    static void BuildInhomogeneousEqnSystemForTriangulation(
        const std::vector<cv::Point3f> &norm_point,
        const std::vector<cv::Mat> &ProjectionMatrix,
        const std::vector<double> &weight,
        cv::Mat &A, cv::Mat &B);
    static void SolveLinearEqn(const cv::Mat &A, const cv::Mat &B, cv::Matx41f &X);

    static void BuildInhomogeneousEqnSystemForTriangulation(
        const cv::Point3f &norm_p1, const cv::Mat &P1,
        const cv::Point3f &norm_p2, const cv::Mat &P2,
        double w1, double w2, cv::Matx43f &A, cv::Matx41f &B);
    static void SolveLinearEqn(const cv::Matx43f &A, const cv::Matx41f &B, cv::Matx41f &X);
};

#endif