#include "../include/eyemodel/estimate_eyeball.hpp"

Estimation_EyeBall::Estimation_EyeBall()
    : flag_setPoint(false), flag_setPointCloud(false), flag_setCalc(false), flag_publish(false) {}

void Estimation_EyeBall::topic_callback_(const geometry_msgs::msg::Transform::SharedPtr msg)
{
    if (flag_setPoint)
        this->input_data(msg);
    if (flag_setCalc)
    {
        this->estimate();
        this->estimate_ceres();
    }
}

void Estimation_EyeBall::input_data(const geometry_msgs::msg::Transform::SharedPtr msg)
{
    if (!flag_setPoint)
        return;

    // メンバ変数の初期化
    cv::Mat cloud(1, 1, CV_64FC3);
    cloud.at<cv::Vec3d>(0)[0] = msg->translation.x;
    cloud.at<cv::Vec3d>(0)[1] = msg->translation.y;
    cloud.at<cv::Vec3d>(0)[2] = msg->translation.z;
    this->pointcloud.push_back(cloud);

    std::cout << "added point #" << pointcloud.rows << " : " << cloud << std::endl;

    flag_setPoint = false;

    if (pointcloud.rows > THRESHOLD_NUM_POINTS)
        flag_setPointCloud = true;
}

void Estimation_EyeBall::setInputflag()
{
    flag_setPoint = true;
}

void Estimation_EyeBall::setCalcflag()
{
    flag_setCalc = true;
}

int Estimation_EyeBall::getPointCloudNum()
{
    return pointcloud.rows;
}

void Estimation_EyeBall::cancel()
{
    this->pointcloud.pop_back();
}

void Estimation_EyeBall::estimate()
{
    // ある程度点群の数が溜まってからスタート
    if (!flag_setPointCloud)
    {
        std::cout << std::endl
                  << "Few PointCloud " << pointcloud.rows << " < " << THRESHOLD_NUM_POINTS << std::endl
                  << "Please set more!" << std::endl;
        flag_setCalc = false;
        return;
    }

    // ラグランジュ未定乗数法を用いて、挿入孔を通る球の方程式を求める
    cv::Mat A = cv::Mat::zeros(4, 4, CV_64FC1);
    cv::Mat B = cv::Mat::zeros(4, 1, CV_64FC1); // ラグランジュ未定乗数法により導いた線形方程式のもの(詳細は研究ノート)

    // A, Bそれぞれに値を挿入する
    for (int i = 0; i < pointcloud.rows; i++)
    {
        // A
        for (size_t j = 0; j < 3; j++)
        {
            for (size_t k = 0; k < 3; k++)
            {
                A.at<double>(j, k) += pointcloud.at<cv::Vec3d>(i)[j] * pointcloud.at<cv::Vec3d>(i)[k];
            }
        }
        for (size_t j = 0; j < 3; j++)
        {
            A.at<double>(j, 3) += pointcloud.at<cv::Vec3d>(i)[j];
            A.at<double>(3, j) += pointcloud.at<cv::Vec3d>(i)[j];
        }
        A.at<double>(3, 3) += 1.0;

        // B
        for (size_t j = 0; j < 3; j++)
        {
            B.at<double>(j, 0) -= pointcloud.at<cv::Vec3d>(i)[j] * pointcloud.at<cv::Vec3d>(i)[0] * pointcloud.at<cv::Vec3d>(i)[0] +
                                  pointcloud.at<cv::Vec3d>(i)[j] * pointcloud.at<cv::Vec3d>(i)[1] * pointcloud.at<cv::Vec3d>(i)[1] +
                                  pointcloud.at<cv::Vec3d>(i)[j] * pointcloud.at<cv::Vec3d>(i)[2] * pointcloud.at<cv::Vec3d>(i)[2];
        }
        B.at<double>(3, 0) -= pointcloud.at<cv::Vec3d>(i)[0] * pointcloud.at<cv::Vec3d>(i)[0] +
                              pointcloud.at<cv::Vec3d>(i)[1] * pointcloud.at<cv::Vec3d>(i)[1] +
                              pointcloud.at<cv::Vec3d>(i)[2] * pointcloud.at<cv::Vec3d>(i)[2];
    }

    cv::Mat tmp_X;
    cv::solve(A, B, tmp_X, cv::DECOMP_SVD);

    double x_est, y_est, z_est, r_est;
    x_est = -tmp_X.at<double>(0, 0) / 2.0;
    y_est = -tmp_X.at<double>(1, 0) / 2.0;
    z_est = -tmp_X.at<double>(2, 0) / 2.0;
    r_est = std::sqrt((x_est * x_est + y_est * y_est + z_est * z_est) - tmp_X.at<double>(3, 0));

    std::cout << std::endl
              << "Estimated Position : " << x_est << ", " << y_est << ", " << z_est << std::endl
              << "Estimated radius : " << r_est << std::endl;

    if (r_est < 0.014 && r_est > 0.01)
    {
        // 大きさ
        eye_shape.Scale.at<double>(0) = r_est * 2.;
        eye_shape.Scale.at<double>(1) = r_est * 2.;
        eye_shape.Scale.at<double>(2) = r_est * 2.;

        // 位置
        eye_shape.Position.at<double>(0) = x_est;
        eye_shape.Position.at<double>(1) = y_est;
        eye_shape.Position.at<double>(2) = z_est;

        flag_publish = true;
    }
}

void Estimation_EyeBall::estimate_ceres()
{
    // ある程度点群の数が溜まってからスタート
    if (!flag_setPointCloud)
    {
        std::cout << std::endl
                  << "Few PointCloud " << pointcloud.rows << " < " << THRESHOLD_NUM_POINTS << std::endl
                  << "Please set more!" << std::endl;
        flag_setCalc = false;
        return;
    }

    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;
    //バンドル調整用パラメータ
    double ellipse_param[3];
    double ellipse_center[3];
    // 初期値
    ellipse_param[0] = 0.012;
    ellipse_param[1] = 0.012;
    ellipse_param[2] = 0.012;
    ellipse_center[0] = pointcloud.at<cv::Vec3d>(0)[0];
    ellipse_center[1] = pointcloud.at<cv::Vec3d>(0)[1];
    ellipse_center[2] = pointcloud.at<cv::Vec3d>(0)[2];

    // コスト関数
    for (int i = 0; i < pointcloud.rows; i++)
    {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<EllipseResiduals, 1, 3, 3>(
                                     new EllipseResiduals(pointcloud.at<cv::Vec3d>(i)[0], pointcloud.at<cv::Vec3d>(i)[1], pointcloud.at<cv::Vec3d>(i)[2])),
                                 NULL, ellipse_param, ellipse_center);
    }
    // 制約設定
    problem.SetParameterLowerBound(ellipse_param, 0, 0.01);
    problem.SetParameterUpperBound(ellipse_param, 0, 0.015);
    problem.SetParameterLowerBound(ellipse_param, 1, 0.01);
    problem.SetParameterUpperBound(ellipse_param, 1, 0.015);
    problem.SetParameterLowerBound(ellipse_param, 2, 0.01);
    problem.SetParameterUpperBound(ellipse_param, 2, 0.015);

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 8;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // eye_shapeに代入
    eye_shape.Position.at<double>(0) = ellipse_center[0];
    eye_shape.Position.at<double>(1) = ellipse_center[1];
    eye_shape.Position.at<double>(2) = ellipse_center[2];
    eye_shape.Scale.at<double>(0) = ellipse_param[0];
    eye_shape.Scale.at<double>(1) = ellipse_param[1];
    eye_shape.Scale.at<double>(2) = ellipse_param[2];

    // printf
    std::cout << std::endl
              << "Estimated Position : " << eye_shape.Position << std::endl
              << "Estimated Scale: " << eye_shape.Scale << std::endl;

    flag_setCalc = false;
}

void Estimation_EyeBall::publish()
{
    // if (!flag_publish)
    //     return;

    // auto marker_msg = std::make_unique<visualization_msgs::msg::Marker>(); //set marker
    // rclcpp::Clock::SharedPtr clock = this->get_clock();
    // int id = 0;
    // marker_msg->header.frame_id = "world";
    // marker_msg->header.stamp = clock->now();
    // marker_msg->id = ++id;

    // // 形状
    // marker_msg->type = visualization_msgs::msg::Marker::SPHERE;
    // marker_msg->action = visualization_msgs::msg::Marker::ADD;

    // // 大きさ
    // marker_msg->scale.x = eye_shape.Scale.at<double>(0);
    // marker_msg->scale.y = eye_shape.Scale.at<double>(1);
    // marker_msg->scale.z = eye_shape.Scale.at<double>(2);

    // // 色
    // marker_msg->color.a = 0.3;
    // marker_msg->color.r = 0.0;
    // marker_msg->color.g = 1.0;
    // marker_msg->color.b = 0.0;

    // // 位置・姿勢
    // marker_msg->pose.position.x = eye_shape.Position.at<double>(0);
    // marker_msg->pose.position.y = eye_shape.Position.at<double>(1);
    // marker_msg->pose.position.z = eye_shape.Position.at<double>(2);
    // marker_msg->pose.orientation.x = eye_shape.Orientation.at<double>(0);
    // marker_msg->pose.orientation.y = eye_shape.Orientation.at<double>(1);
    // marker_msg->pose.orientation.z = eye_shape.Orientation.at<double>(2);
    // marker_msg->pose.orientation.w = eye_shape.Orientation.at<double>(3);

    // publisher_->publish(std::move(marker_msg));
}