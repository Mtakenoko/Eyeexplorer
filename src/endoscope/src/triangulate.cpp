#include "../include/endoscope/triangulate.hpp"
#include "../../htl/include/vector.hpp"

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
    for (int itr = 0; itr < ITR_TRIANGULATE; itr++)
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
        bool iter_end = false;
        for (size_t i = 0; i < size; i++)
        {
            // printf("#%zu [weight, p2] = [%lf %lf], abs = %lf\n", i, weight[i], p2[i], std::abs(weight[i] - p2[i]));
            // 一個でもおかしいのがあれば反復計算は引き続き実行
            if (std::abs(weight[i] - p2[i]) < TRI_ITERATIVE_TERM)
            {
                iter_end = true;
            }
        }
        if (iter_end)
            break;

        // 重みを更新
        weight.clear(); // push_back()するので、一旦要素を空にする
        for (size_t i = 0; i < size; i++)
        {
            weight.push_back(p2[i]);
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

cv::Mat Triangulate::triangulation_RANSAC(const std::vector<cv::Point2f> &point,
                                          const std::vector<cv::Mat> &ProjectionMatrix)
{
    // RANSACでの三角測量
    // 2視点の三次元復元をする。N個の視点が入力されているとすればnC2だけの数を計算する
    // その中で各々の三次元復元結果をpush_backしていく
    // すべて計算した後に、その三次元復元結果を見て平均(or中央)値から閾値を超えて遠いときの２組が怪しいとにらむ
    // 2組のうち1つはKFなので、閾値を超えるものが多い場合はそのKFを削除し、一部しか超えない場合はそのframeを削除する

    size_t size = point.size();
    size_t size_PrjMat = ProjectionMatrix.size();
    if (size != size_PrjMat)
    {
        std::cout << "size != size_PrjMat" << std::endl;
        return cv::Mat::ones(4, 1, CV_32FC1);
    }

    if (size == 2)
    {
        cv::Mat point3D_2scene = Triangulate::triangulation(point[0], ProjectionMatrix[0], point[1], ProjectionMatrix[1]);
        return point3D_2scene;
    }

    // 2視点での三次元復元
    std::vector<float> point3D_X;
    std::vector<float> point3D_Y;
    std::vector<float> point3D_Z;
    std::vector<ImagePair> image_pair;
    for (size_t itr = 0; itr < size; itr++)
    {
        for (size_t itr2 = 0; itr2 < size; itr2++)
        {
            if (itr < itr2)
            {
                cv::Mat point3D_2scene = Triangulate::triangulation(point[itr], ProjectionMatrix[itr], point[itr2], ProjectionMatrix[itr2]);
                // std::cout << "point3D : " << point3D_2scene << std::endl;
                point3D_X.push_back(point3D_2scene.at<float>(0));
                point3D_Y.push_back(point3D_2scene.at<float>(1));
                point3D_Z.push_back(point3D_2scene.at<float>(2));
                image_pair.push_back(ImagePair(itr, itr2));
            }
        }
    }

    // 中央値を計算(各点)
    cv::Point3f center_point;
    center_point.x = htl::Vector::median(point3D_X);
    center_point.y = htl::Vector::median(point3D_Y);
    center_point.z = htl::Vector::median(point3D_Z);

    // 閾値より大きなものを除外した入力を再度生成
    std::vector<int> image_error_counter(size);
    std::vector<bool> image_out_counter(size);
    for (size_t i = 0; i < image_pair.size(); i++)
    {
        float distance_X = std::fabs(point3D_X[i] - center_point.x);
        float distance_Y = std::fabs(point3D_Y[i] - center_point.y);
        float distance_Z = std::fabs(point3D_Z[i] - center_point.z);
        // printf("(#%d #%d)distance [%f %f %f]\n", image_pair[i].first_image_ID, image_pair[i].second_image_ID, distance_X, distance_Y, distance_Z);
        if (distance_X > DISTANCE_CENTER ||
            distance_Y > DISTANCE_CENTER ||
            distance_Z > DISTANCE_CENTER)
        {
            image_error_counter[image_pair[i].first_image_ID]++;
            image_error_counter[image_pair[i].second_image_ID]++;
        }
        if (distance_Z > DISTANCE_CENTER_Z)
        {
            image_out_counter[image_pair[i].first_image_ID] = true;
            image_out_counter[image_pair[i].second_image_ID] = true;
        }
    }

    std::vector<cv::Point2f> point2D;
    std::vector<cv::Mat> PrjMat;
    for (size_t i = 0; i < size; i++)
    {
        if (image_error_counter[i] < (int)((size - 1) / 2 + 1) && image_out_counter[i] == false)
        {
            point2D.push_back(point[i]);
            PrjMat.push_back(ProjectionMatrix[i]);
        }
        else if (image_error_counter[i] < (int)(size / 4 + 1) && image_out_counter[i] == true)
        {
            point2D.push_back(point[i]);
            PrjMat.push_back(ProjectionMatrix[i]);
        }
    }
    if (point2D.size() < 2)
    {
        std::cout << "point2D.size() = " << point2D.size() << std::endl;
        // cv::Mat ans = Triangulate::triangulation(point, ProjectionMatrix);
        cv::Mat ans;
        return ans;
    }
    cv::Mat ans = Triangulate::triangulation(point2D, PrjMat);
    return ans;
}

cv::Mat Triangulate::triangulation_RANSAC(const std::vector<cv::Point2f> &point,
                                          const std::vector<cv::Mat> &ProjectionMatrix,
                                          std::vector<bool> &eliminated_scene,
                                          const size_t min_reconstruction_scene)
{
    // RANSACでの三角測量
    // 2視点の三次元復元をする。N個の視点が入力されているとすればnC2だけの数を計算する
    // その中で各々の三次元復元結果をpush_backしていく
    // すべて計算した後に、その三次元復元結果を見て平均(or中央)値から閾値を超えて遠いときの２組が怪しいとにらむ
    // 2組のうち1つはKFなので、閾値を超えるものが多い場合はそのKFを削除し、一部しか超えない場合はそのframeを削除する

    size_t size = point.size();
    if (size != ProjectionMatrix.size())
    {
        std::cout << "size != size_PrjMat" << std::endl;
        cv::Mat ans;
        return ans;
    }

    if (size < min_reconstruction_scene)
    {
        std::cout << "size < min_reconstruction_scene" << std::endl;
        cv::Mat ans;
        return ans;
    }

    if (size == 2)
    {
        cv::Mat point3D_2scene = Triangulate::triangulation(point[0], ProjectionMatrix[0], point[1], ProjectionMatrix[1]);
        for (size_t i = 0; i < size; i++)
            eliminated_scene.push_back(false);
        return point3D_2scene;
    }

    // 2視点での三次元復元
    std::vector<float> point3D_X;
    std::vector<float> point3D_Y;
    std::vector<float> point3D_Z;
    std::vector<ImagePair> image_pair;
    for (size_t itr = 0; itr < size; itr++)
    {
        for (size_t itr2 = 0; itr2 < size; itr2++)
        {
            if (itr < itr2)
            {
                cv::Mat point3D_2scene = Triangulate::triangulation(point[itr], ProjectionMatrix[itr], point[itr2], ProjectionMatrix[itr2]);
                std::cout << "point3D : " << point3D_2scene << std::endl;
                point3D_X.push_back(point3D_2scene.at<float>(0));
                point3D_Y.push_back(point3D_2scene.at<float>(1));
                point3D_Z.push_back(point3D_2scene.at<float>(2));
                image_pair.push_back(ImagePair(itr, itr2));
            }
        }
    }

    // 中央値を計算(各点)
    cv::Point3f center_point;
    center_point.x = htl::Vector::median(point3D_X);
    center_point.y = htl::Vector::median(point3D_Y);
    center_point.z = htl::Vector::median(point3D_Z);

    std::cout << "center_point : " << center_point << std::endl;

    // 閾値より大きなものを除外した入力を再度生成
    std::vector<cv::Point2f> point2D;
    std::vector<cv::Mat> PrjMat;
    std::vector<int> image_error_counter(size);
    std::vector<bool> image_out_counter(size);
    for (size_t i = 0; i < image_pair.size(); i++)
    {
        float distance_X = std::fabs(point3D_X[i] - center_point.x);
        float distance_Y = std::fabs(point3D_Y[i] - center_point.y);
        float distance_Z = std::fabs(point3D_Z[i] - center_point.z);
        printf("(#%d #%d)distance [%f %f %f]\n", image_pair[i].first_image_ID, image_pair[i].second_image_ID, distance_X, distance_Y, distance_Z);
        if (distance_X > DISTANCE_CENTER ||
            distance_Y > DISTANCE_CENTER ||
            distance_Z > DISTANCE_CENTER)
        {
            image_error_counter[image_pair[i].first_image_ID]++;
            image_error_counter[image_pair[i].second_image_ID]++;
        }
        if (distance_Z > DISTANCE_CENTER_Z)
        {
            image_out_counter[image_pair[i].first_image_ID] = true;
            image_out_counter[image_pair[i].second_image_ID] = true;
        }
    }
    for (size_t i = 0; i < size; i++)
    {
        // printf("image_error_counter[%zu] : %d\n", i, image_error_counter[i]);
        if (image_error_counter[i] < (int)((size - 1) / 2 + 1) && image_out_counter[i] == false)
        {
            printf("#%zu push_back(1)\n", i);
            point2D.push_back(point[i]);
            PrjMat.push_back(ProjectionMatrix[i]);
            eliminated_scene.push_back(false);
        }
        else if (image_error_counter[i] < (int)(size / 4 + 1) && image_out_counter[i] == true)
        {
            printf("#%zu push_back(2)\n", i);
            point2D.push_back(point[i]);
            PrjMat.push_back(ProjectionMatrix[i]);
            eliminated_scene.push_back(false);
        }
        else if (size < min_reconstruction_scene)
        {
            printf("#%zu push_back(3)\n", i);
            point2D.push_back(point[i]);
            PrjMat.push_back(ProjectionMatrix[i]);
            eliminated_scene.push_back(false);
        }
        else
        {
            printf("#%zu is incorrect(4)\n", i);
            eliminated_scene.push_back(true);
        }
    }

    // 除外したもので三角測量
    if (point2D.size() < min_reconstruction_scene)
    {
        // std::cout << "point2D.size() = " << point2D.size() << std::endl;
        // cv::Mat ans = Triangulate::triangulation(point, ProjectionMatrix);
        cv::Mat ans;
        return ans;
    }
    cv::Mat ans = Triangulate::triangulation(point2D, PrjMat);
    // std::cout << "ans point3D : " << ans << std::endl;
    return ans;
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