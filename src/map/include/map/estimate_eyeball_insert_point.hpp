#ifndef ESTIMATION_EYEBALL_HPP__
#define ESTIMATION_EYEBALL_HPP__

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <opencv2/opencv.hpp>

#define THRESHOLD_NUM_POINTS 10

class Eye_Shape
{
public:
    Eye_Shape()
    {
        Orientation = (cv::Mat_<float>(4, 1) << 0., 0., 0., 1.);
        Position = cv::Mat::zeros(3, 1, CV_32FC1);
        Scale = (cv::Mat_<float>(3, 1) << 0.024, 0.024, 0.024);
    };
    cv::Mat Orientation;
    cv::Mat Position;
    cv::Mat Scale;
};

class Estimation_EyeBall : public rclcpp::Node
{
public:
    Estimation_EyeBall();
    Eye_Shape eye_shape;
    cv::Mat pointcloud;
    cv::Point3f insert_point;

private:
    void initialize();
    void process();
    void input_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud);
    void input_marker_data(const visualization_msgs::msg::Marker::SharedPtr msg_pointcloud);
    void estimate();
    void publish();
    void topic_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud);
    void topic_callback2_(const visualization_msgs::msg::Marker::SharedPtr msg_pointcloud);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_insertpoint_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;

    bool flag_SetInsertPoint;
    bool flag_SetPointCloud;
    bool flag_publish;
};

Estimation_EyeBall::Estimation_EyeBall()
    : Node("eyeball_estimator_insertion_point"), flag_SetInsertPoint(false), flag_SetPointCloud(false), flag_publish(false)
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pointcloud/filtered_hold", qos,
        std::bind(&Estimation_EyeBall::topic_callback_, this, std::placeholders::_1));
    subscription_insertpoint_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/insert_point", qos,
        std::bind(&Estimation_EyeBall::topic_callback2_, this, std::placeholders::_1));
}

void Estimation_EyeBall::topic_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud)
{
    Estimation_EyeBall::input_data(msg_pointcloud);
    Estimation_EyeBall::estimate();
    Estimation_EyeBall::publish();
}

void Estimation_EyeBall::topic_callback2_(const visualization_msgs::msg::Marker::SharedPtr msg_marker)
{
    Estimation_EyeBall::input_marker_data(msg_marker);
}

void Estimation_EyeBall::input_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud)
{
    // メンバ変数の初期化
    cv::Mat cloud(msg_pointcloud->width, 1, CV_32FC3);
    auto floatData = reinterpret_cast<float *>(msg_pointcloud->data.data());
    for (uint32_t i = 0; i < msg_pointcloud->width; i++)
    {
        for (uint32_t j = 0; j < 3; j++)
        {
            cloud.at<cv::Vec3f>(i)[j] = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + j];
        }
    }
    pointcloud = cloud.clone();
    flag_SetPointCloud = true;
}

void Estimation_EyeBall::input_marker_data(const visualization_msgs::msg::Marker::SharedPtr msg_pointcloud)
{
    insert_point.x = (float)msg_pointcloud->pose.position.x;
    insert_point.y = (float)msg_pointcloud->pose.position.y;
    insert_point.z = (float)msg_pointcloud->pose.position.z;
    flag_SetInsertPoint = true;
}

void Estimation_EyeBall::estimate()
{
    // 点群、挿入孔がセットされており、かつある程度点群の数が溜まってからスタート
    if (flag_SetPointCloud && flag_SetInsertPoint &&
        pointcloud.rows < THRESHOLD_NUM_POINTS)
    {
        std::cout << "Pointcloud is few. ( " << pointcloud.rows << " points now)" << std::endl;
        return;
    }

    // ラグランジュ未定乗数法を用いて、挿入孔を通る球の方程式を求める
    cv::Mat A = cv::Mat::zeros(5, 5, CV_32FC1);
    cv::Mat B = cv::Mat::zeros(5, 1, CV_32FC1); // ラグランジュ未定乗数法により導いた線形方程式のもの(詳細は研究ノート)

    // A, Bそれぞれに値を挿入する
    for (int i = 0; i < pointcloud.rows; i++)
    {
        // A
        for (size_t j = 0; j < 3; j++)
        {
            for (size_t k = 0; k < 3; k++)
            {
                A.at<float>(j, k) += pointcloud.at<cv::Vec3f>(i)[j] * pointcloud.at<cv::Vec3f>(i)[k];
            }
        }
        for (size_t j = 0; j < 3; j++)
        {
            A.at<float>(j, 3) += pointcloud.at<cv::Vec3f>(i)[j];
            A.at<float>(3, j) += pointcloud.at<cv::Vec3f>(i)[j];
        }
        A.at<float>(3, 3) += 1.0;

        // B
        for (size_t j = 0; j < 3; j++)
        {
            B.at<float>(j, 0) -= pointcloud.at<cv::Vec3f>(i)[j] * pointcloud.at<cv::Vec3f>(i)[0] * pointcloud.at<cv::Vec3f>(i)[0] +
                                 pointcloud.at<cv::Vec3f>(i)[j] * pointcloud.at<cv::Vec3f>(i)[1] * pointcloud.at<cv::Vec3f>(i)[1] +
                                 pointcloud.at<cv::Vec3f>(i)[j] * pointcloud.at<cv::Vec3f>(i)[2] * pointcloud.at<cv::Vec3f>(i)[2];
        }
        B.at<float>(3, 0) -= pointcloud.at<cv::Vec3f>(i)[0] * pointcloud.at<cv::Vec3f>(i)[0] +
                             pointcloud.at<cv::Vec3f>(i)[1] * pointcloud.at<cv::Vec3f>(i)[1] +
                             pointcloud.at<cv::Vec3f>(i)[2] * pointcloud.at<cv::Vec3f>(i)[2];
    }
    // A
    A.at<float>(0, 4) = insert_point.x / 2.0;
    A.at<float>(1, 4) = insert_point.y / 2.0;
    A.at<float>(2, 4) = insert_point.z / 2.0;
    A.at<float>(3, 4) = 1.0 / 2.0;
    A.at<float>(4, 0) = insert_point.x;
    A.at<float>(4, 1) = insert_point.y;
    A.at<float>(4, 2) = insert_point.z;
    A.at<float>(4, 3) = 1.0;
    A.at<float>(4, 4) = 0.0;

    // B
    B.at<float>(4, 0) = -(insert_point.x * insert_point.x +
                          insert_point.y * insert_point.y +
                          insert_point.z * insert_point.z);

    cv::Mat tmp_X;
    cv::solve(A, B, tmp_X, cv::DECOMP_SVD);

    float x_est, y_est, z_est, r_est;
    x_est = -tmp_X.at<float>(0, 0) / 2.0;
    y_est = -tmp_X.at<float>(1, 0) / 2.0;
    z_est = -tmp_X.at<float>(2, 0) / 2.0;
    r_est = std::sqrt((x_est * x_est + y_est * y_est + z_est * z_est) - tmp_X.at<float>(3, 0));

    printf("Estimation Pos [%f %f %f], r : %f\n", x_est, y_est, z_est, r_est);

    if (true /*r_est < 0.04 && r_est > 0.001*/)
    {
        // 大きさ
        eye_shape.Scale.at<float>(0) = r_est * 2.;
        eye_shape.Scale.at<float>(1) = r_est * 2.;
        eye_shape.Scale.at<float>(2) = r_est * 2.;

        // 位置
        eye_shape.Position.at<float>(0) = x_est;
        eye_shape.Position.at<float>(1) = y_est;
        eye_shape.Position.at<float>(2) = z_est;

        flag_publish = true;
    }
}

void Estimation_EyeBall::publish()
{
    if (!flag_publish)
        return;

    auto marker_msg = std::make_unique<visualization_msgs::msg::Marker>(); //set marker
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    int id = 0;
    marker_msg->header.frame_id = "world";
    marker_msg->header.stamp = clock->now();
    marker_msg->ns = "eye_ball";
    marker_msg->id = ++id;

    // 形状
    marker_msg->type = visualization_msgs::msg::Marker::SPHERE;
    marker_msg->action = visualization_msgs::msg::Marker::ADD;

    // 大きさ
    marker_msg->scale.x = (double)eye_shape.Scale.at<float>(0);
    marker_msg->scale.y = (double)eye_shape.Scale.at<float>(1);
    marker_msg->scale.z = (double)eye_shape.Scale.at<float>(2);

    // 色
    marker_msg->color.a = 0.3;
    marker_msg->color.r = 0.0;
    marker_msg->color.g = 1.0;
    marker_msg->color.b = 0.0;

    // 位置・姿勢
    marker_msg->pose.position.x = (double)eye_shape.Position.at<float>(0);
    marker_msg->pose.position.y = (double)eye_shape.Position.at<float>(1);
    marker_msg->pose.position.z = (double)eye_shape.Position.at<float>(2);
    marker_msg->pose.orientation.x = (double)eye_shape.Orientation.at<float>(0);
    marker_msg->pose.orientation.y = (double)eye_shape.Orientation.at<float>(1);
    marker_msg->pose.orientation.z = (double)eye_shape.Orientation.at<float>(2);
    marker_msg->pose.orientation.w = (double)eye_shape.Orientation.at<float>(3);

    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("EyeBall", 10);
    publisher_->publish(std::move(marker_msg));
}
#endif
