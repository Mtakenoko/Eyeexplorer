#include "../include/endoscope/triangulate.hpp"

Triangulate::Triangulate()
{
}

Triangulate::~Triangulate()
{
}

cv::Mat Triangulate::triangulation(const std::vector<cv::Point2f> &point,
                                   const std::vector<cv::Mat> &ProjectionMatrix)
{
    cv::Mat A, B;
    cv::Matx41f X;
    std::vector<cv::Point3f> norm_point;
    std::vector<double> weight;

    size_t size = point.size();
    size_t size_PrjMat = ProjectionMatrix.size();
    if (size != size_PrjMat)
    {
        std::cout << "size != size_PrjMat" << std::endl;
        return cv::Mat::ones(4, 1, CV_32FC1);
    }

    for (size_t i = 0; i < size; i++)
    {
        cv::Point3f p;
        p.x = point[i].x;
        p.y = point[i].y;
        p.z = 1.0;
        norm_point.push_back(p);

        double w;
        w = 1.0;
        weight.push_back(w);
    }
    Triangulate::BuildInhomogeneousEqnSystemForTriangulation(norm_point, ProjectionMatrix, weight, A, B);
    Triangulate::SolveLinearEqn(A, B, X);

    // 反復計算による三次元復元の高精度化
    for (int i = 0; i < ITR_TRIANGULATE; i++)
    {
        std::vector<cv::Mat> PrjMat;
        for (size_t t = 0; t < size; t++)
        {
            cv::Mat prjMat(3, 4, CV_32FC1);
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    prjMat.at<float>(i, j) = ProjectionMatrix[t].at<float>(i, j);
                }
            }
            PrjMat.push_back(prjMat);
        }

        // 重みを計算
        std::vector<double> p2;
        for (size_t i = 0; i < size; i++)
        {
            double p;
            cv::Mat X_mat(X);
            cv::Mat temp = ProjectionMatrix[i].row(2) * X_mat;
            p = temp.at<float>(0);
            p2.push_back(p);
        }

        // 反復を終了するか判定
        bool iter_end = true;
        for (size_t i = 0; i < size; i++)
        {
            // printf("[weight, p2] = [%lf %lf]\n", weight[i], p2[i]);
            // 一個でもおかしいのがあれば反復計算は引き続き実行
            if (std::abs(weight[i] - p2[i]) > TRI_ITERATIVE_TERM)
            {
                iter_end = false;
            }
        }
        if (iter_end)
            break;

        // 重みを更新
        weight.clear(); // push_back()するので、一旦要素を空にする
        for (size_t i = 0; i < size; i++)
        {
            double w = p2[i];
            weight.push_back(w);
        }

        Triangulate::BuildInhomogeneousEqnSystemForTriangulation(norm_point, ProjectionMatrix, weight, A, B);
        Triangulate::SolveLinearEqn(A, B, X);
    }

    cv::Mat ans_X(3, 1, CV_32FC1);
    ans_X.at<float>(0) = X(0);
    ans_X.at<float>(1) = X(1);
    ans_X.at<float>(2) = X(2);
    return ans_X;
}

void Triangulate::BuildInhomogeneousEqnSystemForTriangulation(
    const std::vector<cv::Point3f> &norm_point,
    const std::vector<cv::Mat> &ProjectionMatrix,
    const std::vector<double> &weight,
    cv::Mat &A, cv::Mat &B)
{
    size_t size_point = norm_point.size();
    size_t size_PrjMat = ProjectionMatrix.size();
    if (size_point != size_PrjMat)
    {
        std::cout << "norm_point.size() != ProjectionMatrix.size()" << std::endl;
        return;
    }
    // Aは2n*3の行列
    cv::Mat A_(2 * size_point, 3, CV_32FC1);
    for (size_t i = 0; i < size_point; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            A_.at<float>(2 * i, j) = (norm_point[i].x * ProjectionMatrix[i].at<float>(2, j) - ProjectionMatrix[i].at<float>(0, j)) / weight[i];
            A_.at<float>(2 * i + 1, j) = (norm_point[i].y * ProjectionMatrix[i].at<float>(2, j) - ProjectionMatrix[i].at<float>(1, j)) / weight[i];
        }
    }

    // Bは2n*1の行列
    cv::Mat B_(2 * size_point, 1, CV_32FC1);
    for (size_t i = 0; i < size_PrjMat; i++)
    {
        B_.at<float>(2 * i, 0) = -(norm_point[i].x * ProjectionMatrix[i].at<float>(2, 3) - ProjectionMatrix[i].at<float>(0, 3)) / weight[i];
        B_.at<float>(2 * i + 1, 0) = -(norm_point[i].y * ProjectionMatrix[i].at<float>(2, 3) - ProjectionMatrix[i].at<float>(1, 3)) / weight[i];
    }

    A = A_.clone();
    B = B_.clone();
}

void Triangulate::SolveLinearEqn(const cv::Mat &A, const cv::Mat &B, cv::Matx41f &X)
{
    cv::Mat tmp_X;
    cv::solve(A, B, tmp_X, cv::DECOMP_SVD);
    X(0) = tmp_X.at<float>(0, 0);
    X(1) = tmp_X.at<float>(1, 0);
    X(2) = tmp_X.at<float>(2, 0);
    X(3) = 1.0;
}

cv::Mat Triangulate::triangulation(const cv::Point2f &pnt1, const cv::Mat &PrjMat1,
                                   const cv::Point2f &pnt2, const cv::Mat &PrjMat2)
{
    double w1(1.0), w2(1.0);
    cv::Matx43f A;
    cv::Matx41f B, X;
    cv::Point3f norm_pnt1, norm_pnt2;
    norm_pnt1.x = pnt1.x;
    norm_pnt1.y = pnt1.y;
    norm_pnt1.z = 1.0;
    norm_pnt2.x = pnt2.x;
    norm_pnt2.y = pnt2.y;
    norm_pnt2.z = 1.0;

    Triangulate::BuildInhomogeneousEqnSystemForTriangulation(norm_pnt1, PrjMat1, norm_pnt2, PrjMat2,
                                                             w1, w2, A, B);
    Triangulate::SolveLinearEqn(A, B, X);

    // Iteratively refine triangulation.
    for (int i = 0; i < 10; i++)
    {
        cv::Matx34f P1, P2;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                P1(i, j) = PrjMat1.at<float>(i, j);
                P2(i, j) = PrjMat2.at<float>(i, j);
            }
        }
        // Calculate weight.
        double p2x1 = (P1.row(2) * X)(0);
        double p2x2 = (P2.row(2) * X)(0);

        if (std::abs(w1 - p2x1) < TRI_ITERATIVE_TERM && std::abs(w2 - p2x2) < TRI_ITERATIVE_TERM)
        {
            break;
        }

        w1 = p2x1;
        w2 = p2x2;

        Triangulate::BuildInhomogeneousEqnSystemForTriangulation(norm_pnt1, PrjMat1, norm_pnt2, PrjMat2,
                                                                 w1, w2, A, B);

        Triangulate::SolveLinearEqn(A, B, X);
    }
    cv::Mat ans_X(3, 1, CV_32FC1);
    ans_X.at<float>(0) = X(0);
    ans_X.at<float>(1) = X(1);
    ans_X.at<float>(2) = X(2);
    return ans_X;
}

void Triangulate::BuildInhomogeneousEqnSystemForTriangulation(
    const cv::Point3f &norm_p1, const cv::Mat &P1,
    const cv::Point3f &norm_p2, const cv::Mat &P2,
    double w1, double w2, cv::Matx43f &A, cv::Matx41f &B)
{
    cv::Matx43f A_((norm_p1.x * P1.at<float>(2, 0) - P1.at<float>(0, 0)) / w1,
                   (norm_p1.x * P1.at<float>(2, 1) - P1.at<float>(0, 1)) / w1,
                   (norm_p1.x * P1.at<float>(2, 2) - P1.at<float>(0, 2)) / w1,
                   (norm_p1.y * P1.at<float>(2, 0) - P1.at<float>(1, 0)) / w1,
                   (norm_p1.y * P1.at<float>(2, 1) - P1.at<float>(1, 1)) / w1,
                   (norm_p1.y * P1.at<float>(2, 2) - P1.at<float>(1, 2)) / w1,
                   (norm_p2.x * P2.at<float>(2, 0) - P2.at<float>(0, 0)) / w2,
                   (norm_p2.x * P2.at<float>(2, 1) - P2.at<float>(0, 1)) / w2,
                   (norm_p2.x * P2.at<float>(2, 2) - P2.at<float>(0, 2)) / w2,
                   (norm_p2.y * P2.at<float>(2, 0) - P2.at<float>(1, 0)) / w2,
                   (norm_p2.y * P2.at<float>(2, 1) - P2.at<float>(1, 1)) / w2,
                   (norm_p2.y * P2.at<float>(2, 2) - P2.at<float>(1, 2)) / w2);

    cv::Matx41f B_(-(norm_p1.x * P1.at<float>(2, 3) - P1.at<float>(0, 3)) / w1,
                   -(norm_p1.y * P1.at<float>(2, 3) - P1.at<float>(1, 3)) / w1,
                   -(norm_p2.x * P2.at<float>(2, 3) - P2.at<float>(0, 3)) / w2,
                   -(norm_p2.y * P2.at<float>(2, 3) - P2.at<float>(1, 3)) / w2);

    A = A_;
    B = B_;
}

void Triangulate::SolveLinearEqn(const cv::Matx43f &A, const cv::Matx41f &B,
                                 cv::Matx41f &X)
{
    cv::Matx31f tmp_X;
    tmp_X(0) = X(0);
    tmp_X(1) = X(1);
    tmp_X(2) = X(2);

    cv::solve(A, B, tmp_X, cv::DECOMP_SVD);

    X(0) = tmp_X(0);
    X(1) = tmp_X(1);
    X(2) = tmp_X(2);
    X(3) = 1.0;
}