#ifndef ESTIMATION_INSERT_POINT_HPP__
#define ESTIMATION_INSERT_POINT_HPP__

#include <fstream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "/home/takeyama/workspace/htl/opencv/transform.hpp"

#define QUEUE_START_SIZE 5
#define QUEUE_HOLD_SIZE 10
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
    float now_d, pre_d;
    float p;
    int count;
    std::ofstream outputfile;

public:
    Correct() : count(0), outputfile("/home/takeyama/workspace/ros2_eyeexplorer/src/arm/arm_status/output/test.txt")
    {
        outputfile << "output.x" << " " << "output.y" << " " << "output.z" << " " << "Pw.x" << " " << "Pw.y" << " " << "Pw.z" << " " << "n.x" << " " << "n.y" << " " << "n.z" << std::endl;
    }
    ~Correct()
    {
        outputfile.close();
    }
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
        pre_n = input_pre_n;
        cv::Point3f pre_distance = input_pre_Pp - pre_Pw;
        pre_d = std::sqrt(pre_distance.x * pre_distance.x + pre_distance.y * pre_distance.y + pre_distance.z * pre_distance.z);
        pre_Pp = pre_Pw - pre_d * pre_n;
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
        Pp = Pw - (pre_d * pre_n.dot(n) + delta_Pw.dot(n)) * n;
        delta_Pp = Pp - pre_Pp;

        // 移動量を視線方向に垂直な平面に正射影
        cv::Point3f delta_PwT = delta_Pw - n.dot(delta_Pw) * n;
        cv::Point3f delta_PpT = delta_Pp - n.dot(delta_Pp) * n;

        // 更新式
        float update = p * delta_Pp.dot(n);
        float update_ = p * delta_Pp.dot(pre_n);
        float update2 = p * 1000. * delta_PwT.dot(delta_PpT);
        now_d = pre_d * pre_n.dot(n) + delta_Pw.dot(n) + update;
        cv::Point3f output = Pw - now_d * n;

        if (count < 10 || count % 100 == 0)
        {
            std::cout << std::endl;
            // std::cout << "now_d : " << now_d << std::endl;
            // std::cout << "pre_d : " << pre_d << std::endl;
            // std::cout << "pre_d * pre_n.dot(n) : " << pre_d * pre_n.dot(n) << std::endl;
            // std::cout << "delta_Pw.dot(n) : " << delta_Pw.dot(n) << std::endl;
            // std::cout << "pre_d * pre_n.dot(n) - delta_Pw.dot(n) : " << pre_d * pre_n.dot(n) - delta_Pw.dot(n) << std::endl;
            // std::cout << "update : " << update << std::endl;
            // std::cout << "update_ : " << update_ << std::endl;
            // std::cout << "update2 : " << update2 << std::endl;
            std::cout << "output : " << output << std::endl;
            outputfile << output.x << " " << output.y << " " << output.z << " " << Pw.x << " " << Pw.y << " " << Pw.z << " " << n.x << " " << n.y << " " << n.z << std::endl;
        }

        // 過去の値を更新
        pre_Pw = Pw;
        pre_Pp = Pp;
        pre_d = now_d;
        pre_n = n;

        count++;

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
    bool flag_correct;
};

Estimation_InsertPoint::Estimation_InsertPoint()
    : Node("insertpoint_estimator"), flag_correct(false)
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("insert_point", qos);
    subscription_ = this->create_subscription<geometry_msgs::msg::Transform>("endoscope_transform", qos, std::bind(&Estimation_InsertPoint::topic_callback_, this, std::placeholders::_1));

    correction.setParameter(0.8);
}

void Estimation_InsertPoint::topic_callback_(const geometry_msgs::msg::Transform::SharedPtr msg_pointcloud)
{
    this->input_data(msg_pointcloud);
    if (trans_vector.size() >= QUEUE_START_SIZE)
    {
        if (flag_correct)
        {
            output_point_shape.Position = correction.calc(now_intersect.point, now_intersect.normal);
            output_point_shape.Orientation = insert_point_shape.Orientation;
            output_point_shape.Scale = insert_point_shape.Scale;
            flag_correct = true;
        }
        else
        {
            this->calcIntersection();
            correction.setInitialData(pre_inter.point, pre_inter.normal, insert_point_shape.Position);
            flag_correct = true;
        }
        this->publish();
    }
}

void Estimation_InsertPoint::input_data(const geometry_msgs::msg::Transform::SharedPtr msg_pointcloud)
{
    if (msg_pointcloud->translation.x == 0. || msg_pointcloud->translation.y == 0. || msg_pointcloud->translation.z == 0.)
        return;

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

        if (this->trans_vector.size() > QUEUE_START_SIZE)
        {
            flag_correct = true;
            if (this->trans_vector.size() > QUEUE_HOLD_SIZE)
            {
                std::vector<Line_Intersect>::iterator itr_begin = this->trans_vector.begin();
                this->trans_vector.erase(itr_begin);
            }
        }
        else
        {
            flag_correct = false;
        }

        // std::cout << std::endl;
        // std::cout << "trans size : " << trans_vector.size() << std::endl;
        // std::cout << "point : " << now_intersect.point << std::endl;
        // std::cout << "Rotation : " << now_intersect.Rotation << std::endl;
        // std::cout << "tilt : " << now_intersect.tilt << std::endl;
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
