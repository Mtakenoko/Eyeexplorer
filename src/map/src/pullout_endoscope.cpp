// 内視鏡の抜去を行う
// publish: TS~01のDO
// subscribe: ①内視鏡座標　②三次元復元した点群
#include "../include/map/pullout_endoscope.hpp"
#include "../../HTL/include/transform.h"

Transform transform;

PullOut::PullOut()
    : flag_pull(false)
{
    printf("Start PullOut\n");
}

void PullOut::topic_callback_(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg_pointcloud,
                              const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                              rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud)
{
    // 初期化
    this->initialize();

    // subscribeしたデータを取得(同時に空っぽなら処理を中断させる)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    EndoscopePose endoscopepose;
    if (this->input_data(msg_pointcloud, msg_arm, cloud_, &endoscopepose))
        return;

    // メイン処理
    this->process(cloud_, endoscopepose);

    // publish
    this->publish(pub_pointcloud);
}

void PullOut::process(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                      const EndoscopePose endoscopePose)
{
    // 中身が空っぽか確認
    if (this->chace_empty(point_cloud))
        return;

    // 点群から平面を推定する
    this->estimate_plane(point_cloud);
    // this->estimate_sphere(point_cloud);

    // 内視鏡と平面の距離を計算する
    float distance = this->calc_distance(endoscopePose);

    // 距離が閾値以内に入ったらpull
    if (distance < SAFETY_DISTANCE)
    {
        flag_pull = true;
    }
}

void PullOut::initialize()
{
    flag_pull = false;
}

void PullOut::publish(const std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> &pub_pointcloud)
{
    auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pub_pointcloud->publish(std::move(msg_cloud_pub));
}

bool PullOut::input_data(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg_pointcloud,
                         const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_,
                         EndoscopePose *endoscopePose)
{
    // 点群データ
    point_cloud_->width = msg_pointcloud->width;
    point_cloud_->height = msg_pointcloud->height;
    point_cloud_->is_dense = false;
    if (point_cloud_->width <= 1)
    {
        return true;
    }

    std::vector<uint8_t> data_box = msg_pointcloud->data;
    const auto floatData = reinterpret_cast<float *>(data_box.data());
    for (uint32_t i = 0; i < msg_pointcloud->width; ++i)
    {
        pcl::PointXYZ xyz;
        xyz.x = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + 0];
        xyz.y = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + 1];
        xyz.z = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + 2];
        point_cloud_->points.push_back(xyz);
    }

    // 内視鏡の位置・姿勢
    cv::Mat Rotation_world = transform.QuaternionToRotMat(msg_arm->rotation.x, msg_arm->rotation.y, msg_arm->rotation.z, msg_arm->rotation.w);
    cv::Mat Transform_world = (cv::Mat_<float>(3, 1) << msg_arm->translation.x, msg_arm->translation.y, msg_arm->translation.z);
    cv::cv2eigen(Rotation_world, endoscopePose->Rotation);
    cv::cv2eigen(Transform_world, endoscopePose->Transform);

    return false;
}

bool PullOut::chace_empty(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    size_t point_num = point_cloud->points.size();
    if (point_num == 0)
        return true;
    else
        return false;
}

void PullOut::estimate_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    // 点群情報から3次元平面を推定する
    // ax+by+cz=dの4パラメータを最小二乗法にて推定(方法については研究ノートを参照のこと)
    printf("\nEstimeate plane\n");
    // 点群の重心を求める（点群の平均値と同値）
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*point_cloud, xyz_centroid);
    printf("compute3DCentroid : [%f %f %f]\n", xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

    // 各点における重心との距離を求める
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> X;
    X.resize(point_cloud->points.size(), 3);
    for (size_t i = 0; i < point_cloud->points.size(); i++)
    {
        X(i, 0) = point_cloud->points[i].x - xyz_centroid[0]; //X座標の移動
        X(i, 1) = point_cloud->points[i].y - xyz_centroid[1]; //Y座標の移動
        X(i, 2) = point_cloud->points[i].z - xyz_centroid[2]; //Z座標の移動
    }
    //
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A = X.transpose() * X;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> eigensolver(A);
    std::cout << "固有ベクトル：\n"
              << eigensolver.eigenvectors() << std::endl;
    std::cout << "固有値：\n"
              << eigensolver.eigenvalues() << std::endl;

    Eigen::Vector3f minEigenVec;
    if (eigensolver.eigenvalues()[0] < eigensolver.eigenvalues()[1])
    {
        if (eigensolver.eigenvalues()[0] < eigensolver.eigenvalues()[2])
        {
            minEigenVec = eigensolver.eigenvectors().row(0);
        }
        else
        {
            minEigenVec = eigensolver.eigenvectors().row(2);
        }
    }
    else
    {
        if (eigensolver.eigenvalues()[1] < eigensolver.eigenvalues()[2])
        {
            minEigenVec = eigensolver.eigenvectors().row(1);
        }
        else
        {
            minEigenVec = eigensolver.eigenvectors().row(2);
        }
    }
    std::cout << "最小固有値の固有ベクトル：\n"
              << minEigenVec << std::endl;

    // 固有ベクトル上に点群の重心が通るように平面パラメータを定める。
    double est_plane[4];
    est_plane[0] = minEigenVec[0];
    est_plane[1] = minEigenVec[1];
    est_plane[2] = minEigenVec[2];
    est_plane[3] = -(est_plane[0] * xyz_centroid[0] + est_plane[1] * xyz_centroid[1] + est_plane[2] * xyz_centroid[2]);

    printf("平面の方程式 : %f x + %f y + %f z + %f = 0\n", est_plane[0], est_plane[1], est_plane[2], est_plane[3]);
}

void PullOut::estimate_sphere(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    // 点群情報から3次元の球体を推定する
    // (x-a)^2+(y-b)^2+(z-c)^2=r^2を最小二乗法にて推定
    printf("point.hesight = %d\n", point_cloud->height);
}

float PullOut::calc_distance(const EndoscopePose endoscopePose)
{
    float distance;
    distance = std::abs(endoscopePose.Transform(0));
    return distance;
}
