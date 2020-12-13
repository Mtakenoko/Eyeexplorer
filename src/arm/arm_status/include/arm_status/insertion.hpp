#ifndef ESTIMATION_INSERT_POINT_HPP__
#define ESTIMATION_INSERT_POINT_HPP__

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "/home/takeyama/workspace/htl/opencv/transform.hpp"

#define QUEUE_START_SIZE 5
// #define QUEUE_MAX_SIZE 10
#define PHI_MAX_CHOOSE 0.1

struct Line_Intersect
{
public:
    cv::Vec3f tilt;
    cv::Point3f point;
    cv::Mat Rotation;
    cv::Point3f normal;
};

struct Shape
{
public:
    cv::Vec4f Orientation;
    cv::Point3f Position;
    cv::Vec3f Scale;

    Line_Intersect tranform_Line_Intersect()
    {
        Line_Intersect LI;
        LI.point.x = this->Position.x;
        LI.point.y = this->Position.y;
        LI.point.z = this->Position.z;
        cv::Mat R = htl::Transform::QuaternionToRotMat<float>(this->Orientation[0], this->Orientation[1], this->Orientation[2], this->Orientation[3]);
        LI.tilt[0] = R.at<float>(0, 2);
        LI.tilt[1] = R.at<float>(1, 2);
        LI.tilt[2] = R.at<float>(2, 2);
        LI.normal.x = LI.tilt[0];
        LI.normal.y = LI.tilt[1];
        LI.normal.z = LI.tilt[2];
        LI.Rotation = R.clone();
        return LI;
    }
};

class Correct
{
private:
    cv::Point3f Pw, pre_Pw; // 内視鏡先端位置
    cv::Point3f Pp, pre_Pp; // 挿入孔位置（推定）
    cv::Point3f n, pre_n;   // 内視鏡視線方向
    cv::Point3f delta_Pw, delta_Pp;
    cv::Point3f delta_PwT, delta_PpT;
    float now_d, pre_d;
    float p;

public:
    void setInitialData(const cv::Point3f &input_pre_Pw, const cv::Point3f input_pre_n, const cv::Point3f &input_pre_Pp)
    {
        pre_Pw = input_pre_Pw;
        pre_n = input_pre_n;
        cv::Point3f pre_distance = input_pre_Pp - pre_Pw;
        pre_d = std::sqrt(pre_distance.x * pre_distance.x + pre_distance.y * pre_distance.y + pre_distance.z * pre_distance.z);
        pre_Pp = pre_Pw - pre_d * pre_n;
    }
    void setPreData(const cv::Point3f &input_pre_Pw, const cv::Point3f &input_pre_Pp, const cv::Point3f input_pre_n)
    {
        pre_Pw = input_pre_Pw;
        pre_Pp = input_pre_Pp;
        pre_n = input_pre_n;
        cv::Point3f pre_distance = pre_Pp - pre_Pw;
        pre_d = std::sqrt(pre_distance.x * pre_distance.x + pre_distance.y * pre_distance.y + pre_distance.z * pre_distance.z);
    }
    void setParameter(const float input_p)
    {
        p = input_p;
    }
    cv::Point3f calc(const cv::Point3f &input_Pw, const cv::Point3f &input_n)
    {
        // 現在の位置をinput
        Pw = input_Pw;
        n = input_n;

        // ロボットの位置の移動量
        delta_Pw = Pw - pre_Pw;
        // 計算上求まる挿入孔位置（これだけだと過去の長さの誤差の影響をそのまま受けるのでよくない）
        Pp = Pw - (delta_Pw.dot(n) + pre_d * pre_n.dot(n)) * n;
        delta_Pp = Pp - pre_Pp;

        // 移動量を視線方向に垂直な平面に正射影
        delta_PwT = delta_Pw - n.dot(delta_Pw) * n;
        delta_PpT = delta_Pp - n.dot(delta_Pp) * n;

        // 更新式
        float update = p * delta_PwT.dot(delta_PpT);
        now_d = delta_Pw.dot(n) + pre_d * pre_n.dot(n) + update;
        cv::Point3f output = Pw - now_d * n;

        // std::cout << std::endl;
        // std::cout << "Pw : " << Pw << std::endl;
        // std::cout << "pre_Pw : " << pre_Pw << std::endl;
        // std::cout << "Pp : " << Pp << std::endl;
        // std::cout << "pre_Pp : " << pre_Pp << std::endl;
        // std::cout << "n : " << n << std::endl;
        // std::cout << "pre_n : " << pre_n << std::endl;
        // std::cout << "delta_PwT" << delta_PwT << std::endl;
        // std::cout << "delta_PpT" << delta_PpT << std::endl;
        // std::cout << "delta_Pw.dot(n) : " << delta_Pw.dot(n) << std::endl;
        // std::cout << "pre_d * pre_n.dot(n) : " << pre_d * pre_n.dot(n) << std::endl;
        // std::cout << "update : " << update << std::endl;
        // std::cout << "now_d : " << now_d << std::endl;
        // std::cout << "pre_d : " << pre_d << std::endl;
        // std::cout << "output : " << output << std::endl;

        // 過去の値を更新
        cv::Point3f pre_distance = pre_Pw - output;
        pre_d = std::sqrt(pre_distance.x * pre_distance.x + pre_distance.y * pre_distance.y + pre_distance.z * pre_distance.z);
        pre_Pw = pre_Pw;
        pre_Pp = output;
        pre_n = pre_distance / pre_d;

        return output;
    }
};

class Estimation_InsertPoint : public rclcpp::Node
{
public:
    Estimation_InsertPoint();
    Shape insert_point_shape;
    Shape output_point_shape;
    Line_Intersect now_intersect;
    Line_Intersect pre_inter;
    Correct correction;
    cv::Mat pointcloud;

private:
    void initialize();
    void process();
    void input_data(const geometry_msgs::msg::Transform::SharedPtr msg_pointcloud);
    void estimate();
    void publish();
    void topic_callback_(const geometry_msgs::msg::Transform::SharedPtr msg_pointcloud);
    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;

private:
    void calcIntersection();
    std::vector<Line_Intersect> trans_vector;
    bool flag_calc, flag_correct;
};

Estimation_InsertPoint::Estimation_InsertPoint()
    : Node("insertpoint_estimator"), flag_calc(false), flag_correct(false)
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("insert_point", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Transform>("endoscope_transform", qos, std::bind(&Estimation_InsertPoint::topic_callback_, this, std::placeholders::_1));

    correction.setParameter(100.0);
}

void Estimation_InsertPoint::topic_callback_(const geometry_msgs::msg::Transform::SharedPtr msg_pointcloud)
{
    this->initialize();
    this->input_data(msg_pointcloud);
    if (trans_vector.size() >= QUEUE_START_SIZE)
    {
        if (flag_calc)
        {
            this->calcIntersection();
            correction.setInitialData(pre_inter.point, pre_inter.normal, insert_point_shape.Position);
        }
        else
        {
            if (flag_correct)
            {
                output_point_shape.Position = correction.calc(now_intersect.point, now_intersect.normal);
                output_point_shape.Orientation = insert_point_shape.Orientation;
                output_point_shape.Scale = insert_point_shape.Scale;
            }
        }
        this->publish();
    }
}

void Estimation_InsertPoint::initialize()
{
}

void Estimation_InsertPoint::input_data(const geometry_msgs::msg::Transform::SharedPtr msg_pointcloud)
{
    Shape msg_trans;
    msg_trans.Position.x = (float)msg_pointcloud->translation.x;
    msg_trans.Position.y = (float)msg_pointcloud->translation.y;
    msg_trans.Position.z = (float)msg_pointcloud->translation.z;
    msg_trans.Orientation[0] = (float)msg_pointcloud->rotation.x;
    msg_trans.Orientation[1] = (float)msg_pointcloud->rotation.y;
    msg_trans.Orientation[2] = (float)msg_pointcloud->rotation.z;
    msg_trans.Orientation[3] = (float)msg_pointcloud->rotation.w;
    now_intersect = msg_trans.tranform_Line_Intersect();

    // 仰角
    cv::Mat rot_now = htl::Transform::QuaternionToRotMat<float>(msg_trans.Orientation[0], msg_trans.Orientation[1], msg_trans.Orientation[2], msg_trans.Orientation[3]);

    bool near_frame_detect(false);
    std::vector<float> phi_vector;
    // 新しく登録したキーフレームから探索する(すぐ見つかりやすいので高速になる)
    for (auto itr = trans_vector.begin(); itr != trans_vector.end(); itr++)
    {
        float phi = htl::Transform::RevFromRotMat<float>(itr->Rotation.t() * rot_now);
        phi_vector.push_back(phi);
        near_frame_detect = std::abs(phi) < PHI_MAX_CHOOSE;
    }
    if (!near_frame_detect)
    {
        trans_vector.push_back(now_intersect);
        pre_inter = now_intersect;

        std::cout << std::endl;
        std::cout << "trans size : " << trans_vector.size() << std::endl;
        std::cout << "point : " << now_intersect.point << std::endl;
        std::cout << "Rotation : " << now_intersect.Rotation << std::endl;
        std::cout << "tilt : " << now_intersect.tilt << std::endl;

        flag_calc = true;
        if (this->trans_vector.size() > QUEUE_START_SIZE)
        {
            std::vector<Line_Intersect>::iterator itr_begin = this->trans_vector.begin();
            this->trans_vector.erase(itr_begin);
            flag_calc = false;
        }
    }
}

void Estimation_InsertPoint::calcIntersection()
{
    cv::Mat A(trans_vector.size() + 3, trans_vector.size() + 3, CV_32FC1);
    for (size_t i = 0; i < trans_vector.size(); i++)
        for (size_t j = 0; j < trans_vector.size(); j++)
            if (i == j)
                A.at<float>(i, j) = trans_vector[i].tilt.dot(trans_vector[i].tilt);
            else
                A.at<float>(i, j) = 0.0;
    for (size_t i = trans_vector.size(); i < trans_vector.size() + 3; i++)
        for (size_t j = trans_vector.size(); j < trans_vector.size() + 3; j++)
            if (i == j)
                A.at<float>(i, j) = (float)trans_vector.size();
            else
                A.at<float>(i, j) = 0.0;
    for (size_t i = 0; i < trans_vector.size(); i++)
        for (size_t j = trans_vector.size(); j < trans_vector.size() + 3; j++)
        {
            A.at<float>(i, j) = -trans_vector[i].tilt[j - trans_vector.size()];
            A.at<float>(j, i) = -trans_vector[i].tilt[j - trans_vector.size()];
        }

    cv::Mat B(trans_vector.size() + 3, 1, CV_32FC1);
    for (size_t i = 0; i < trans_vector.size(); i++)
        B.at<float>(i, 0) = -trans_vector[i].tilt.dot(trans_vector[i].point);
    float temp_x(0), temp_y(0), temp_z(0);
    for (size_t i = 0; i < trans_vector.size(); i++)
    {
        temp_x += trans_vector[i].point.x;
        temp_y += trans_vector[i].point.y;
        temp_z += trans_vector[i].point.z;
    }
    B.at<float>(trans_vector.size(), 0) = temp_x;
    B.at<float>(trans_vector.size() + 1, 0) = temp_y;
    B.at<float>(trans_vector.size() + 2, 0) = temp_z;

    cv::Mat tmp_X;
    cv::solve(A, B, tmp_X, cv::DECOMP_SVD);
    insert_point_shape.Position.x = tmp_X.at<float>(trans_vector.size(), 0);
    insert_point_shape.Position.y = tmp_X.at<float>(trans_vector.size() + 1, 0);
    insert_point_shape.Position.z = tmp_X.at<float>(trans_vector.size() + 2, 0);

    std::cout << "Position : " << insert_point_shape.Position << std::endl;
    // std::cout << "A : " << A << std::endl;
    // std::cout << "B : " << B << std::endl;

    flag_calc = false;
    flag_correct = true;
}

void Estimation_InsertPoint::publish()
{
    auto marker_msg = std::make_unique<visualization_msgs::msg::Marker>(); //set marker
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    int id = 0;
    marker_msg->header.frame_id = "world";
    marker_msg->header.stamp = clock->now();
    marker_msg->ns = "insertion_point";
    marker_msg->id = ++id;

    // 形状
    marker_msg->type = visualization_msgs::msg::Marker::SPHERE;
    marker_msg->action = visualization_msgs::msg::Marker::ADD;

    // 大きさ
    marker_msg->scale.x = 0.005;
    marker_msg->scale.y = 0.005;
    marker_msg->scale.z = 0.005;

    // 色
    marker_msg->color.a = 1.0;
    marker_msg->color.r = 1.0;
    marker_msg->color.g = 0.0;
    marker_msg->color.b = 0.0;

    // 位置・姿勢
    marker_msg->pose.position.x = (double)output_point_shape.Position.x;
    marker_msg->pose.position.y = (double)output_point_shape.Position.y;
    marker_msg->pose.position.z = (double)output_point_shape.Position.z;
    marker_msg->pose.orientation.x = 0.0;
    marker_msg->pose.orientation.y = 0.0;
    marker_msg->pose.orientation.z = 0.0;
    marker_msg->pose.orientation.w = 1.0;

    publisher_->publish(std::move(marker_msg));
}
#endif
