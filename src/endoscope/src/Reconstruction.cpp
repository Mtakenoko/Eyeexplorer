#include "../include/endoscope/Reconstruction.hpp"
#include "../../HTL/include/transform.h"
#include "../../HTL/include/msg_converter.h"

Transform transform;
Converter converter;

Reconstruction::Reconstruction()
    : flag_reconstruction(true), flag_setFirstFrame(true), flag_showImage(true), flag_estimate_move(true),
      threshold_knn_ratio(0.7f), threshold_ransac(5.0)
{
    if (flag_showImage)
    {
        //ウィンドウの用意
        cv::namedWindow("matching_image", cv::WINDOW_AUTOSIZE);
        // cv::namedWindow("nomatching_image", cv::WINDOW_AUTOSIZE);
    }
}

void Reconstruction::initialize()
{
    knn_matches.clear();
    dmatch.clear();
    inliners_matches.clear();
    frame_data.extractor.point.clear();
    frame_data.extractor.keypoints.clear();
    keyframe_data.extractor.point.clear();
    keyframe_data.extractor.keypoints.clear();
    matched_point1.clear();
    matched_point2.clear();
    frame_data.camerainfo.CameraMatrix = this->CameraMat.clone();
}

void Reconstruction::setFirstFrame()
{
    frame_data.camerainfo.Rotation = Rotation_eye.clone();
    frame_data.camerainfo.Transform = Transform_zeros.clone();
    keyframe_database.push_back(frame_data);
}

void Reconstruction::chooseKeyFrame()
{
    if (keyframe_database.size() >= 10)
    {
        // 新しく登録したキーフレームから探索する
        for (auto itr = keyframe_database.end() - 1; itr != keyframe_database.begin() - 1; --itr)
        {
            // 判定条件1: Z方向の変化が少ない
            // カメラ座標での移動量の計算
            cv::Mat t_endo = itr->camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - itr->camerainfo.Transform_world);
            if (abs(t_endo.at<float>(2)) < 0.005)
            {
                // 判定条件2: xy方向の変化or仰角の変化が一定範囲内にある
                // xy方向の移動量
                cv::Point2f t_move_xy(t_endo.at<float>(0), t_endo.at<float>(1));
                bool moving_xy = cv::norm(t_move_xy) < 0.05 && cv::norm(t_move_xy) > 0.01;

                // 仰角
                float phi = transform.RevFromRotMat(itr->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);
                bool moving_phi = abs(phi) < 0.1 && abs(phi) > 0.001;
                if (moving_xy && moving_phi)
                {
                    // printf("あったぞ＾＾(xy: %f, phi: %f)\n", cv::norm(t_move_xy), abs(phi));
                    keyframe_data = *itr;
                    return;
                }
            }
            // printf("t_endo_z = %f\n", abs(t_endo.at<float>(2)));
            // printf("move_xy = %f, phi = %f\n", cv::norm(t_move_xy), abs(phi));
            // printf("戻るぞ\n");
        }
        // 何一つ当てはまるのがなければ
        // 三次元復元は行わず、KFの候補であれば登録だけはする
        // printf("一つもなかったよ；；\n");
        flag_reconstruction = false;
        this->setKeyFrame();
        return;
    }
    else
    {
        keyframe_data = keyframe_database[keyframe_database.size() - 1];
    }
}

void Reconstruction::setKeyFrame()
{
    if (this->checkKeyFrame())
    {
        printf("KeyFrame was setted!\n");
        frame_data.camerainfo.Rotation = Rotation_eye.clone();
        frame_data.camerainfo.Transform = Transform_zeros.clone();
        keyframe_database.push_back(frame_data);
    }
}

bool Reconstruction::checkKeyFrame()
{
    // KeyFrameに新たに登録してからのフレーム間隔
    auto itr_endo = keyframe_database.end() - 1;
    int frame_span = frame_data.camerainfo.frame_num - itr_endo->camerainfo.frame_num;

    // xy方向の移動量
    cv::Mat t_move = itr_endo->camerainfo.Rotation_world.t() * frame_data.camerainfo.Transform_world;
    cv::Point2f t_move_xy;
    t_move_xy.x = t_move.at<float>(0);
    t_move_xy.y = t_move.at<float>(1);
    float t_move_norm = cv::norm(t_move_xy);

    // 仰角
    float phi = transform.RevFromRotMat(itr_endo->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);

    if (frame_span > 10 && (t_move_norm > 3. || phi > M_PI / 540.))
        return true;
    return false;
}

int Reconstruction::encoding2mat_type(const std::string &encoding)
{
    if (encoding == "mono8")
    {
        return CV_8UC1;
    }
    else if (encoding == "bgr8")
    {
        return CV_8UC3;
    }
    else if (encoding == "mono16")
    {
        return CV_16SC1;
    }
    else if (encoding == "rgba8")
    {
        return CV_8UC4;
    }
    else if (encoding == "bgra8")
    {
        return CV_8UC4;
    }
    else if (encoding == "32FC1")
    {
        return CV_32FC1;
    }
    else if (encoding == "rgb8")
    {
        return CV_8UC3;
    }
    else
    {
        throw std::runtime_error("Unsupported encoding type");
    }
}
std::vector<cv::Point2f> Reconstruction::keypoint2Point(std::vector<cv::KeyPoint> kp)
{
    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < kp.size(); i++)
    {
        points.push_back(kp[i].pt);
    }
    return points;
}
void Reconstruction::input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                                const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm)
{
    // Subscribeした画像
    cv::Mat frame_image(msg_image->height, msg_image->width, encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    if (msg_image->encoding == "rgb8")
    {
        cv::cvtColor(frame_image, frame_image, cv::COLOR_RGB2BGR);
    }
    frame_data.extractor.image = frame_image.clone();

    // 運動学で求めたグローバル座標からみたカメラの位置姿勢
    frame_data.camerainfo.Rotation_world = transform.QuaternionToRotMat(msg_arm->rotation.x, msg_arm->rotation.y, msg_arm->rotation.z, msg_arm->rotation.w);
    frame_data.camerainfo.Transform_world = (cv::Mat_<float>(3, 1) << msg_arm->translation.x, msg_arm->translation.y, msg_arm->translation.z);

    // 画像のID情報
    frame_data.camerainfo.frame_num = atoi(msg_image->header.frame_id.c_str());

    // 特徴点検出・特徴量記述
    frame_data.extractor.extractAndcompute(Extractor::DetectorType::AKAZE);
}

void Reconstruction::knn_matching()
{
    if (keyframe_database.empty())
    {
        printf("    keyframe_database is empty!\n");
        return;
    }
    // descriptorはCV_32FじゃないとFLANNでのマッチングはできないらしい
    if (frame_data.extractor.descirptors.type() != CV_32F)
        frame_data.extractor.descirptors.convertTo(frame_data.extractor.descirptors, CV_32F);
    if (keyframe_data.extractor.descirptors.type() != CV_32F)
        keyframe_data.extractor.descirptors.convertTo(keyframe_data.extractor.descirptors, CV_32F);
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    matcher->knnMatch(frame_data.extractor.descirptors, keyframe_data.extractor.descirptors, knn_matches, 2);
}

void Reconstruction::BF_matching()
{
    if (keyframe_database.empty())
    {
        printf("    keyframe_database is empty!\n");
        return;
    }
    // descriptorはCV_32FじゃないとFLANNでのマッチングはできないらしい
    if (frame_data.extractor.descirptors.type() != CV_32F)
        frame_data.extractor.descirptors.convertTo(frame_data.extractor.descirptors, CV_32F);
    if (keyframe_data.extractor.descirptors.type() != CV_32F)
        keyframe_data.extractor.descirptors.convertTo(keyframe_data.extractor.descirptors, CV_32F);
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    matcher->match(frame_data.extractor.descirptors, keyframe_data.extractor.descirptors, dmatch);
}

void Reconstruction::knn_outlier_remover()
{
    // 誤対応除去①：knnマッチングでなるべく差が大きいものだけを選択
    // 距離が小さいほうがよりマッチング度合いが高い
    if (knn_matches.size() == 0)
    {
        printf("knn_matches.size() == 0\n");
        return;
    }

    std::vector<cv::Point2f> match_point1, match_point2;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < threshold_knn_ratio * knn_matches[i][1].distance)
        {
            match_point1.push_back(frame_data.extractor.keypoints[knn_matches[i][0].queryIdx].pt);
            match_point2.push_back(keyframe_data.extractor.keypoints[knn_matches[i][0].trainIdx].pt);
            dmatch.push_back(knn_matches[i][0]);
        }
    }

    // 誤対応除去②：ホモグラフィ変換を行うときのRANSACを用いる
    cv::Mat homography, inliner_mask;
    std::vector<cv::KeyPoint> inliners1_keypoints, inliners2_keypoints;
    std::vector<int> inliners_idx;
    homography = cv::findHomography(match_point1, match_point2, cv::RANSAC, threshold_ransac, inliner_mask);
    inliners_idx.clear();
    for (int i = 0; i < inliner_mask.rows; i++)
    {
        if (inliner_mask.at<uchar>(i))
        {
            // inliners1_keypoints.push_back(matched1_keypoints[i]);
            // inliners2_keypoints.push_back(matched2_keypoints[i]);
            inliners_matches.push_back(dmatch[i]);
            inliners_idx.push_back(i); // インデックスを整理した後にもオリジナルのインデックスを参照できるように保存
            matched_point1.push_back(frame_data.extractor.keypoints[dmatch[i].queryIdx].pt);
            matched_point2.push_back(keyframe_data.extractor.keypoints[dmatch[i].trainIdx].pt);
        }
    }
    match_num = inliners_matches.size();
}

void Reconstruction::BF_outlier_remover()
{
    if (dmatch.size() < 5)
    {
        printf("dmatch.size() == %zu\n", dmatch.size());
        return;
    }

    std::vector<cv::Point2f> match_point1, match_point2;
    for (size_t i = 0; i < dmatch.size(); ++i)
    {
        match_point1.push_back(frame_data.extractor.keypoints[dmatch[i].queryIdx].pt);
        match_point2.push_back(keyframe_data.extractor.keypoints[dmatch[i].trainIdx].pt);
    }

    // 誤対応除去：ホモグラフィ変換を行うときのRANSACを用いる
    std::vector<int> inliners_idx;
    cv::Mat homography, inliner_mask;
    std::vector<cv::KeyPoint> inliners1_keypoints, inliners2_keypoints;
    inliners_idx.clear();
    homography = cv::findHomography(match_point1, match_point2, inliner_mask, cv::RANSAC, 5.);
    for (int i = 0; i < inliner_mask.rows; i++)
    {
        if (inliner_mask.at<uchar>(i))
        {
            inliners_matches.push_back(dmatch[i]);
            inliners_idx.push_back(i); // インデックスを整理した後にもオリジナルのインデックスを参照できるように保存
            matched_point1.push_back(frame_data.extractor.keypoints[dmatch[i].queryIdx].pt);
            matched_point2.push_back(keyframe_data.extractor.keypoints[dmatch[i].trainIdx].pt);
        }
    }
    match_num = inliners_matches.size();
}

void Reconstruction::triangulation()
{
    // カメラ座標の計算
    frame_data.camerainfo.Rotation = keyframe_data.camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world;
    frame_data.camerainfo.Transform = keyframe_data.camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - keyframe_data.camerainfo.Transform_world);
    // ↑なぜか距離を7倍くらいずらしてやると実寸と合うっぽい
    // @TODO: このバグ修正
    keyframe_data.camerainfo.Rotation = Rotation_eye.clone();
    keyframe_data.camerainfo.Transform = Transform_zeros.clone();
    frame_data.camerainfo.setData();
    keyframe_data.camerainfo.setData();
    frame_data.extractor.match2point_query(inliners_matches);
    keyframe_data.extractor.match2point_train(inliners_matches);

    // 三角測量
    std::cout << "triangulation" << std::endl;
    cv::Mat p3, p3_arm;
    for (size_t i = 0; i < match_num; i++)
    {
        cv::Mat point4D(4, match_num, CV_32FC1);
        cv::Mat point3D_result, point3D_result_arm;
        cv::triangulatePoints(keyframe_data.camerainfo.ProjectionMatrix, frame_data.camerainfo.ProjectionMatrix,
                              cv::Mat(keyframe_data.extractor.point[i]), cv::Mat(frame_data.extractor.point[i]),
                              point4D);
        cv::convertPointsFromHomogeneous(point4D.reshape(4, 1), point3D_result);
        cv::Mat point3D_mine = triangulate(keyframe_data.extractor.point[i], keyframe_data.camerainfo.ProjectionMatrix,
                                           frame_data.extractor.point[i], frame_data.camerainfo.ProjectionMatrix);
        // point3D_result_arm = keyframe_data.camerainfo.Rotation_world * point3D_result.reshape(1, 3) + keyframe_data.camerainfo.Transform_world;
        point3D_result_arm = point3D_result.clone();
        p3.push_back(point3D_result);
        p3_arm.push_back(point3D_result_arm.reshape(3, 1));

        // 復元した点を再投影
        /* ここから */
        float point[3];
        point[0] = point3D_result_arm.at<float>(0);
        point[1] = point3D_result_arm.at<float>(1);
        point[2] = point3D_result_arm.at<float>(2);

        float pt_keyframe[3];
        pt_keyframe[0] = keyframe_data.camerainfo.Rotation_world.at<float>(0, 0) * point[0] + keyframe_data.camerainfo.Rotation_world.at<float>(1, 0) * point[1] + keyframe_data.camerainfo.Rotation_world.at<float>(2, 0) * point[2];
        pt_keyframe[1] = keyframe_data.camerainfo.Rotation_world.at<float>(0, 1) * point[0] + keyframe_data.camerainfo.Rotation_world.at<float>(1, 1) * point[1] + keyframe_data.camerainfo.Rotation_world.at<float>(2, 1) * point[2];
        pt_keyframe[2] = keyframe_data.camerainfo.Rotation_world.at<float>(0, 2) * point[0] + keyframe_data.camerainfo.Rotation_world.at<float>(1, 2) * point[1] + keyframe_data.camerainfo.Rotation_world.at<float>(2, 2) * point[2];

        float pt_frame[3];
        pt_frame[0] = frame_data.camerainfo.Rotation_world.at<float>(0, 0) * point[0] + frame_data.camerainfo.Rotation_world.at<float>(1, 0) * point[1] + frame_data.camerainfo.Rotation_world.at<float>(2, 0) * point[2];
        pt_frame[1] = frame_data.camerainfo.Rotation_world.at<float>(0, 1) * point[0] + frame_data.camerainfo.Rotation_world.at<float>(1, 1) * point[1] + frame_data.camerainfo.Rotation_world.at<float>(2, 1) * point[2];
        pt_frame[2] = frame_data.camerainfo.Rotation_world.at<float>(0, 2) * point[0] + frame_data.camerainfo.Rotation_world.at<float>(1, 2) * point[1] + frame_data.camerainfo.Rotation_world.at<float>(2, 2) * point[2];

        float t_keyframe[3];
        t_keyframe[0] = keyframe_data.camerainfo.Rotation_world.at<float>(0, 0) * keyframe_data.camerainfo.Transform_world.at<float>(0) + keyframe_data.camerainfo.Rotation_world.at<float>(1, 0) * keyframe_data.camerainfo.Transform_world.at<float>(1) + keyframe_data.camerainfo.Rotation_world.at<float>(2, 0) * keyframe_data.camerainfo.Transform_world.at<float>(2);
        t_keyframe[1] = keyframe_data.camerainfo.Rotation_world.at<float>(0, 1) * keyframe_data.camerainfo.Transform_world.at<float>(0) + keyframe_data.camerainfo.Rotation_world.at<float>(1, 1) * keyframe_data.camerainfo.Transform_world.at<float>(1) + keyframe_data.camerainfo.Rotation_world.at<float>(2, 1) * keyframe_data.camerainfo.Transform_world.at<float>(2);
        t_keyframe[2] = keyframe_data.camerainfo.Rotation_world.at<float>(0, 2) * keyframe_data.camerainfo.Transform_world.at<float>(0) + keyframe_data.camerainfo.Rotation_world.at<float>(1, 1) * keyframe_data.camerainfo.Transform_world.at<float>(1) + keyframe_data.camerainfo.Rotation_world.at<float>(2, 2) * keyframe_data.camerainfo.Transform_world.at<float>(2);

        float t_frame[3];
        t_frame[0] = frame_data.camerainfo.Rotation_world.at<float>(0, 0) * frame_data.camerainfo.Transform_world.at<float>(0) + frame_data.camerainfo.Rotation_world.at<float>(1, 0) * frame_data.camerainfo.Transform_world.at<float>(1) + frame_data.camerainfo.Rotation_world.at<float>(2, 0) * frame_data.camerainfo.Transform_world.at<float>(2);
        t_frame[1] = frame_data.camerainfo.Rotation_world.at<float>(0, 1) * frame_data.camerainfo.Transform_world.at<float>(0) + frame_data.camerainfo.Rotation_world.at<float>(1, 1) * frame_data.camerainfo.Transform_world.at<float>(1) + frame_data.camerainfo.Rotation_world.at<float>(2, 1) * frame_data.camerainfo.Transform_world.at<float>(2);
        t_frame[2] = frame_data.camerainfo.Rotation_world.at<float>(0, 2) * frame_data.camerainfo.Transform_world.at<float>(0) + frame_data.camerainfo.Rotation_world.at<float>(1, 1) * frame_data.camerainfo.Transform_world.at<float>(1) + frame_data.camerainfo.Rotation_world.at<float>(2, 2) * frame_data.camerainfo.Transform_world.at<float>(2);

        float xp_keyframe = (pt_keyframe[0] - t_keyframe[0]) / (pt_keyframe[2] - t_keyframe[2]);
        float yp_keyframe = (pt_keyframe[1] - t_keyframe[1]) / (pt_keyframe[2] - t_keyframe[2]);

        float xp_frame = (pt_frame[0] - t_frame[0]) / (pt_frame[2] - t_frame[2]);
        float yp_frame = (pt_frame[1] - t_frame[1]) / (pt_frame[2] - t_frame[2]);

        const float focal_x = 396.7;
        const float focal_y = 396.9;
        const float u_x = 163.6;
        const float u_y = 157.1;
        float predicted_keyframe[2];
        predicted_keyframe[0] = focal_x * xp_keyframe + u_x;
        predicted_keyframe[1] = focal_y * yp_keyframe + u_y;
        float predicted_frame[2];
        predicted_frame[0] = focal_x * xp_frame + u_x;
        predicted_frame[1] = focal_y * yp_frame + u_y;
        /* ここまで */

        std::cout
            << "ID: " << i << std::endl
            << "keyframe: " << cv::Mat(keyframe_data.extractor.point[i]).reshape(2, 1) << std::endl
            << "frame   :" << cv::Mat(frame_data.extractor.point[i]).reshape(2, 1) << std::endl
            // << "point4D :" << point4D.reshape(4, 1) << std::endl
            << "point3D :" << point3D_result << std::endl
            << "point3D_mine :" << point3D_mine.reshape(3, 1) << std::endl
            << "predict_key :" << predicted_keyframe[0] << ", " << predicted_keyframe[1] << std::endl
            << "predict_fra :" << predicted_frame[0] << ", " << predicted_frame[1] << std::endl;
        std::cout << std::endl;
    }
    point3D = p3.clone();
    point3D_arm = p3_arm.clone();

    std::cout << "R_keyframe" << keyframe_data.camerainfo.Rotation_world << std::endl;
    std::cout << "t_keyframe" << keyframe_data.camerainfo.Transform_world.reshape(3, 1) << std::endl;
    std::cout << "R_frame" << frame_data.camerainfo.Rotation_world << std::endl;
    std::cout << "t_frame" << frame_data.camerainfo.Transform_world.reshape(3, 1) << std::endl;
    std::cout << "R_endo_key" << keyframe_data.camerainfo.Rotation << std::endl;
    std::cout << "t_endo_key" << keyframe_data.camerainfo.Transform << std::endl;
    std::cout << "R_endo" << frame_data.camerainfo.Rotation << std::endl;
    std::cout << "t_endo" << frame_data.camerainfo.Transform.reshape(3, 1) << std::endl;
    std::cout << "Rt_kf" << keyframe_data.camerainfo.CameraPose << std::endl;
    std::cout << "Rt_f" << frame_data.camerainfo.CameraPose << std::endl;
    std::cout << "Prj_kf" << keyframe_data.camerainfo.ProjectionMatrix << std::endl;
    std::cout << "Prj_f" << frame_data.camerainfo.ProjectionMatrix << std::endl;
    // std::cout << "p1" << cv::Mat(keyframe_data.extractor.point) << std::endl;
    // std::cout << "point_key" << keyframe_data.extractor.point[0] << std::endl;
    // std::cout << "point" << frame_data.extractor.point[0] << std::endl;
    // std::cout << "match_num " << match_num << std::endl;
    // std::cout << "p3_arm: " << p3_arm.rows << std::endl;
    std::cout << std::endl;
}

cv::Mat Reconstruction::triangulate(const cv::Point2f &pnt1, const cv::Mat &PrjMat1,
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

    BuildInhomogeneousEqnSystemForTriangulation(norm_pnt1, PrjMat1, norm_pnt2, PrjMat2,
                                                w1, w2, A, B);
    SolveLinearEqn(A, B, X);

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

        BuildInhomogeneousEqnSystemForTriangulation(norm_pnt1, PrjMat1, norm_pnt2, PrjMat2,
                                                    w1, w2, A, B);

        SolveLinearEqn(A, B, X);
    }
    cv::Mat ans_X(3, 1, CV_32FC1);
    ans_X.at<float>(0) = X(0);
    ans_X.at<float>(1) = X(1);
    ans_X.at<float>(2) = X(2);
    return ans_X;
}

void Reconstruction::BuildInhomogeneousEqnSystemForTriangulation(
    const cv::Point3f &norm_p1, const cv::Mat &P1,
    const cv::Point3f &norm_p2, const cv::Mat &P2,
    double w1, double w2, cv::Matx43f &A, cv::Matx41f &B)
{
    cv::Matx43d A_((norm_p1.x * P1.at<float>(2, 0) - P1.at<float>(0, 0)) / w1,
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

    cv::Matx41d B_(-(norm_p1.x * P1.at<float>(2, 3) - P1.at<float>(0, 3)) / w1,
                   -(norm_p1.y * P1.at<float>(2, 3) - P1.at<float>(1, 3)) / w1,
                   -(norm_p2.x * P2.at<float>(2, 3) - P2.at<float>(0, 3)) / w2,
                   -(norm_p2.y * P2.at<float>(2, 3) - P2.at<float>(1, 3)) / w2);

    A = A_;
    B = B_;
}

void Reconstruction::SolveLinearEqn(const cv::Matx43f &A, const cv::Matx41f &B,
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

void Reconstruction::triangulation_test()
{
    //３次元点群の座標位置を設定（８個）
    std::vector<cv::Point3f> point3D;
    point3D.push_back(cv::Point3f(-100, -100, -100));
    point3D.push_back(cv::Point3f(100, -100, -100));
    point3D.push_back(cv::Point3f(100, -100, 100));
    point3D.push_back(cv::Point3f(-100, -100, 100));
    point3D.push_back(cv::Point3f(-100, 100, -100));
    point3D.push_back(cv::Point3f(100, 100, -100));
    point3D.push_back(cv::Point3f(100, 100, 100));
    point3D.push_back(cv::Point3f(-100, 100, 100));

    //カメラの位置を設定
    float hbl = 150, hbly = 0, distance = 800;
    float thetaL = 1.6, thetaR = -0.1;

    //カメラの内部パラメータを設定
    cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 320, 0, 160,
                            0, 320, 160,
                            0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<float>(5, 1) << 0, 0, 0, 0, 0);

    //回転ベクトルを設定
    cv::Mat RotL = (cv::Mat_<float>(3, 3) << std::cos(thetaL), -std::sin(thetaL), 0,
                    std::sin(thetaL), std::cos(thetaL), 0,
                    0, 0, 1);
    cv::Mat RotR = (cv::Mat_<float>(3, 3) << std::cos(thetaR), -std::sin(thetaR), 0,
                    std::sin(thetaR), std::cos(thetaR), 0,
                    0, 0, 1);
    cv::Mat revcL, revcR;
    cv::Rodrigues(RotL, revcL);
    cv::Rodrigues(RotR, revcR);

    //並進ベクトルを設定
    cv::Mat tvecL = (cv::Mat_<float>(3, 1) << hbl, hbly, distance);
    cv::Mat tvecR = (cv::Mat_<float>(3, 1) << -hbl, -hbly, distance);

    //左画像, 右画像に射影する
    std::vector<cv::Point2f> imagePointsL, imagePointsR;
    cv::projectPoints(point3D, revcL, tvecL, cameraMatrix, distCoeffs, imagePointsL);
    cv::projectPoints(point3D, revcR, tvecR, cameraMatrix, distCoeffs, imagePointsR);

    //printf
    std::cout << "imagePointsL" << std::endl;
    std::cout << imagePointsL << std::endl
              << std::endl;
    std::cout << "imagePointsR" << std::endl;
    std::cout << imagePointsR << std::endl
              << std::endl;

    //描画
    cv::Mat dst_imageL = cv::Mat(cv::Size(320, 320), CV_8UC3, cv::Scalar::all(255));
    cv::Mat dst_imageR = cv::Mat(cv::Size(320, 320), CV_8UC3, cv::Scalar::all(255));

    cv::circle(dst_imageL, imagePointsL[0], 2, cv::Scalar(255, 0, 255), -1);
    cv::circle(dst_imageR, imagePointsR[0], 2, cv::Scalar(255, 0, 255), -1);
    for (int i = 1; i < 8; i++)
    {
        cv::circle(dst_imageL, imagePointsL[i], 2, cv::Scalar::all(0), -1);
        cv::circle(dst_imageR, imagePointsR[i], 2, cv::Scalar::all(0), -1);
    }
    for (int i = 0; i < 4; i++)
    {
        cv::line(dst_imageL, imagePointsL[i], imagePointsL[(i + 1) % 4], cv::Scalar::all(0));
        cv::line(dst_imageL, imagePointsL[i + 4], imagePointsL[((i + 1) % 4) + 4], cv::Scalar::all(0));
        cv::line(dst_imageL, imagePointsL[i], imagePointsL[i + 4], cv::Scalar::all(0));
        cv::line(dst_imageR, imagePointsR[i], imagePointsR[(i + 1) % 4], cv::Scalar::all(0));
        cv::line(dst_imageR, imagePointsR[i + 4], imagePointsR[((i + 1) % 4) + 4], cv::Scalar::all(0));
        cv::line(dst_imageR, imagePointsR[i], imagePointsR[i + 4], cv::Scalar::all(0));
    }
    cv::Mat showimage;
    cv::hconcat(dst_imageL, dst_imageR, showimage);
    cv::imshow("L, R画像(基準長 = 300)", showimage);

    // ここから三角測量
    // 画像上の点の位置を少しだけいじってみる
    imagePointsL[0].x += 5;
    imagePointsL[0].y -= 5;
    imagePointsL[1].x += 10;
    imagePointsL[1].y -= 10;
    imagePointsL[2].x += 15;
    imagePointsL[2].y -= 15;
    imagePointsL[3].x += 20;
    imagePointsL[3].y -= 20;

    //左右のカメラの射影行列を設定する
    cv::Mat RtL(3, 4, CV_32FC1), RtR(3, 4, CV_32FC1);
    cv::hconcat(RotL, tvecL, RtL);
    cv::hconcat(RotR, tvecR, RtR);

    cv::Mat projectionMatrixL(3, 4, CV_32FC1), projectionMatrixR(3, 4, CV_32FC1);
    projectionMatrixL = cameraMatrix * RtL;
    projectionMatrixR = cameraMatrix * RtR;

    //三次元復元
    cv::Mat points4D;
    std::vector<cv::Point3f> point3D_result1, point3D_result2, point3D_result3;
    std::cout << "RtL: " << RtL << std::endl;
    std::cout << "RtR: " << RtR << std::endl;
    std::cout << "projectionMatrixL: " << projectionMatrixL << std::endl;
    std::cout << "projectionMatrixR: " << projectionMatrixR << std::endl;
    std::cout << "座標位置(X, Y, Z)" << std::endl;
    for (int i = 0; i < 8; i++)
    {
        cv::triangulatePoints(projectionMatrixL, projectionMatrixR, cv::Mat(imagePointsL[i]), cv::Mat(imagePointsR[i]), points4D);
        cv::convertPointsFromHomogeneous(points4D.reshape(4, 1), point3D_result1);
        std::cout << point3D_result1 << std::endl;
    }
    cv::waitKey(3);
}

void Reconstruction::triangulation_est()
{
    // カメラ座標の計算
    cv::Mat R_endo, t_endo;
    R_est.convertTo(R_endo, CV_32FC1);
    t_est.convertTo(t_endo, CV_32FC1);
    frame_data.camerainfo.Rotation = R_endo.clone();
    frame_data.camerainfo.Transform = t_endo.clone();
    frame_data.camerainfo.setData();
    keyframe_data.camerainfo.setData();
    frame_data.extractor.match2point_query(inliners_matches);
    keyframe_data.extractor.match2point_train(inliners_matches);

    // 三角測量
    std::cout << "triangulation" << std::endl;
    point3D.zeros(match_num, 1, CV_32F);
    for (size_t i = 0; i < match_num; i++)
    {
        cv::Mat point4D(4, match_num, CV_32FC1);
        cv::Mat point3D_result;
        cv::triangulatePoints(keyframe_data.camerainfo.ProjectionMatrix, frame_data.camerainfo.ProjectionMatrix,
                              cv::Mat(keyframe_data.extractor.point[i]), cv::Mat(frame_data.extractor.point[i]),
                              point4D);
        cv::convertPointsFromHomogeneous(point4D.reshape(4, 1), point3D_result);
        point3D.push_back(point3D_result);
    }
    // std::cout << "R_frame" << frame_data.camerainfo.Rotation_world << std::endl;
    // std::cout << "t_frame" << frame_data.camerainfo.Transform_world << std::endl;
    // std::cout << "R_endo" << frame_data.camerainfo.Rotation << std::endl;
    // std::cout << "t_endo" << frame_data.camerainfo.Transform << std::endl;
    // std::cout << "Rt_f" << frame_data.camerainfo.CameraPose << std::endl;
    // std::cout << "Rt_kf" << keyframe_data.camerainfo.CameraPose << std::endl;
    // std::cout << "Prj_kf" << keyframe_data.camerainfo.ProjectionMatrix << std::endl;
    // std::cout << "Prj_f" << frame_data.camerainfo.ProjectionMatrix << std::endl;
    // std::cout << "p1" << cv::Mat(keyframe_data.extractor.point) << std::endl;
    // std::cout << "R_keyframe" << keyframe_data.camerainfo.Rotation_world << std::endl;
    // std::cout << "t_keyframe" << keyframe_data.camerainfo.Transform_world << std::endl;
    // std::cout << "point1" << frame_data.extractor.point << std::endl;
}

void Reconstruction::bundler()
{
    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;
    //バンドル調整用パラメータ
    double mutable_camera_for_observations[2][6];
    double **mutable_point_for_observations = new double *[2 * match_num];
    for (size_t i = 0; i < 2 * match_num; i++)
    {
        mutable_point_for_observations[i] = new double[3];
    }

    cv::Mat rvec_keyframe, rvec_frame;
    cv::Rodrigues(keyframe_data.camerainfo.Rotation_world, rvec_keyframe);
    cv::Rodrigues(frame_data.camerainfo.Rotation_world, rvec_frame);

    //KeyFrameの方の情報
    mutable_camera_for_observations[0][0] = (double)rvec_keyframe.at<float>(0);
    mutable_camera_for_observations[0][1] = (double)rvec_keyframe.at<float>(1);
    mutable_camera_for_observations[0][2] = (double)rvec_keyframe.at<float>(2);
    mutable_camera_for_observations[0][3] = (double)keyframe_data.camerainfo.Transform_world.at<float>(0);
    mutable_camera_for_observations[0][4] = (double)keyframe_data.camerainfo.Transform_world.at<float>(1);
    mutable_camera_for_observations[0][5] = (double)keyframe_data.camerainfo.Transform_world.at<float>(2);
    //Frameの方の情報
    mutable_camera_for_observations[1][0] = (double)rvec_frame.at<float>(0);
    mutable_camera_for_observations[1][1] = (double)rvec_frame.at<float>(1);
    mutable_camera_for_observations[1][2] = (double)rvec_frame.at<float>(2);
    mutable_camera_for_observations[1][3] = (double)frame_data.camerainfo.Transform_world.at<float>(0);
    mutable_camera_for_observations[1][4] = (double)frame_data.camerainfo.Transform_world.at<float>(1);
    mutable_camera_for_observations[1][5] = (double)frame_data.camerainfo.Transform_world.at<float>(2);

    for (size_t i = 0; i < match_num; i++)
    {
        mutable_point_for_observations[i][0] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[0];
        mutable_point_for_observations[i][1] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[1];
        mutable_point_for_observations[i][2] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[2];
        mutable_point_for_observations[i + match_num][0] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[0];
        mutable_point_for_observations[i + match_num][1] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[1];
        mutable_point_for_observations[i + match_num][2] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[2];

        //コスト関数
        // ceres::CostFunction *cost_function_train = SnavelyReprojectionError::Create((double)keyframe_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.x,
        //                                                                             (double)keyframe_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.y);
        // ceres::CostFunction *cost_function_query = SnavelyReprojectionError::Create((double)frame_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.x,
        //                                                                             (double)frame_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.y);
        // problem.AddResidualBlock(cost_function_train, NULL, mutable_camera_for_observations[0], mutable_point_for_observations[i]);
        // problem.AddResidualBlock(cost_function_query, NULL, mutable_camera_for_observations[1], mutable_point_for_observations[i + match_num]);

        ceres::CostFunction *cost_function_train = ProjectionErrorCostFuctor::Create((double)keyframe_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.x,
                                                                                     (double)keyframe_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.y);
        ceres::CostFunction *cost_function_query = ProjectionErrorCostFuctor::Create((double)frame_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.x,
                                                                                     (double)frame_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.y);
        problem.AddResidualBlock(cost_function_train, NULL,
                                 &mutable_camera_for_observations[0][0], &mutable_camera_for_observations[0][1], &mutable_camera_for_observations[0][2],
                                 &mutable_camera_for_observations[0][3], &mutable_camera_for_observations[0][4], &mutable_camera_for_observations[0][5],
                                 &mutable_point_for_observations[i][0], &mutable_point_for_observations[i][1], &mutable_point_for_observations[i][2]);
        problem.AddResidualBlock(cost_function_query, NULL,
                                 &mutable_camera_for_observations[1][0], &mutable_camera_for_observations[1][1], &mutable_camera_for_observations[1][2],
                                 &mutable_camera_for_observations[1][3], &mutable_camera_for_observations[1][4], &mutable_camera_for_observations[1][5],
                                 &mutable_point_for_observations[i + match_num][0], &mutable_point_for_observations[i + match_num][1], &mutable_point_for_observations[i + match_num][2]);
    }

    // 上限下限の設定
    // 1.
    // problem.SetParameterLowerBound(mutable_camera_for_observations[0], 0, (double)rvec_keyframe.at<float>(0) - 0.05);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[0], 1, (double)rvec_keyframe.at<float>(1) - 0.05);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[0], 2, (double)rvec_keyframe.at<float>(2) - 0.05);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[0], 3, (double)keyframe_data.camerainfo.Transform_world.at<float>(0) - 0.01);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[0], 4, (double)keyframe_data.camerainfo.Transform_world.at<float>(1) - 0.01);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[0], 5, (double)keyframe_data.camerainfo.Transform_world.at<float>(2) - 0.01);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[1], 0, (double)rvec_frame.at<float>(0) - 0.05);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[1], 1, (double)rvec_frame.at<float>(1) - 0.05);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[1], 2, (double)rvec_frame.at<float>(2) - 0.05);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[1], 3, (double)frame_data.camerainfo.Transform_world.at<float>(0) - 0.01);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[1], 4, (double)frame_data.camerainfo.Transform_world.at<float>(1) - 0.01);
    // problem.SetParameterLowerBound(mutable_camera_for_observations[1], 5, (double)frame_data.camerainfo.Transform_world.at<float>(2) - 0.01);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[0], 0, (double)rvec_keyframe.at<float>(0) + 0.05);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[0], 1, (double)rvec_keyframe.at<float>(1) + 0.05);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[0], 2, (double)rvec_keyframe.at<float>(2) + 0.05);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[0], 3, (double)keyframe_data.camerainfo.Transform_world.at<float>(0) + 0.01);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[0], 4, (double)keyframe_data.camerainfo.Transform_world.at<float>(1) + 0.01);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[0], 5, (double)keyframe_data.camerainfo.Transform_world.at<float>(2) + 0.01);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[1], 0, (double)rvec_frame.at<float>(0) + 0.05);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[1], 1, (double)rvec_frame.at<float>(1) + 0.05);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[1], 2, (double)rvec_frame.at<float>(2) + 0.05);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[1], 3, (double)frame_data.camerainfo.Transform_world.at<float>(0) + 0.01);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[1], 4, (double)frame_data.camerainfo.Transform_world.at<float>(1) + 0.01);
    // problem.SetParameterUpperBound(mutable_camera_for_observations[1], 5, (double)frame_data.camerainfo.Transform_world.at<float>(2) + 0.01);

    // // 2.
    problem.SetParameterLowerBound(&mutable_camera_for_observations[0][0], 0, (double)rvec_keyframe.at<float>(0) - 0.05);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[0][1], 0, (double)rvec_keyframe.at<float>(1) - 0.05);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[0][2], 0, (double)rvec_keyframe.at<float>(2) - 0.05);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[0][3], 0, (double)keyframe_data.camerainfo.Transform_world.at<float>(0) - 0.01);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[0][4], 0, (double)keyframe_data.camerainfo.Transform_world.at<float>(1) - 0.01);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[0][5], 0, (double)keyframe_data.camerainfo.Transform_world.at<float>(2) - 0.01);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[1][0], 0, (double)rvec_frame.at<float>(0) - 0.05);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[1][1], 0, (double)rvec_frame.at<float>(1) - 0.05);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[1][2], 0, (double)rvec_frame.at<float>(2) - 0.05);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[1][3], 0, (double)frame_data.camerainfo.Transform_world.at<float>(0) - 0.01);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[1][4], 0, (double)frame_data.camerainfo.Transform_world.at<float>(1) - 0.01);
    problem.SetParameterLowerBound(&mutable_camera_for_observations[1][5], 0, (double)frame_data.camerainfo.Transform_world.at<float>(2) - 0.01);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[0][0], 0, (double)rvec_keyframe.at<float>(0) + 0.05);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[0][1], 0, (double)rvec_keyframe.at<float>(1) + 0.05);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[0][2], 0, (double)rvec_keyframe.at<float>(2) + 0.05);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[0][3], 0, (double)keyframe_data.camerainfo.Transform_world.at<float>(0) + 0.01);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[0][4], 0, (double)keyframe_data.camerainfo.Transform_world.at<float>(1) + 0.01);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[0][5], 0, (double)keyframe_data.camerainfo.Transform_world.at<float>(2) + 0.01);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[1][0], 0, (double)rvec_frame.at<float>(0) + 0.05);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[1][1], 0, (double)rvec_frame.at<float>(1) + 0.05);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[1][2], 0, (double)rvec_frame.at<float>(2) + 0.05);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[1][3], 0, (double)frame_data.camerainfo.Transform_world.at<float>(0) + 0.01);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[1][4], 0, (double)frame_data.camerainfo.Transform_world.at<float>(1) + 0.01);
    problem.SetParameterUpperBound(&mutable_camera_for_observations[1][5], 0, (double)frame_data.camerainfo.Transform_world.at<float>(2) + 0.01);

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 8;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //レポートなど出力
    // std::cout << summary.FullReport() << "\n";
    // std::cout << "Use for " << match_num << " points for bundler." << std::endl;
    // std::cout << "rvec_x:" << rvec_frame.at<float>(0) << "->" << mutable_camera_for_observations[1][0] << std::endl;
    // std::cout << "rvec_y:" << rvec_frame.at<float>(1) << "->" << mutable_camera_for_observations[1][1] << std::endl;
    // std::cout << "rvec_z:" << rvec_frame.at<float>(2) << "->" << mutable_camera_for_observations[1][2] << std::endl;
    // std::cout << "Trans_x: " << frame_data.camerainfo.Transform_world.at<float>(0) << "->" << mutable_camera_for_observations[1][3] << std::endl;
    // std::cout << "Trans_y: " << frame_data.camerainfo.Transform_world.at<float>(1) << "->" << mutable_camera_for_observations[1][4] << std::endl;
    // std::cout << "Trans_z: " << frame_data.camerainfo.Transform_world.at<float>(2) << "->" << mutable_camera_for_observations[1][5] << std::endl
    //           << std::endl;
    // for (size_t i = 0; i < match_num; i++)
    // {
    //     std::cout << "point_x: " << point3D_arm.at<cv::Vec3f>(i, 0)[0] << " -> " << mutable_point_for_observations[i][0] << " & " << mutable_point_for_observations[i + match_num][0] << std::endl;
    //     std::cout << "point_y: " << point3D_arm.at<cv::Vec3f>(i, 0)[1] << " -> " << mutable_point_for_observations[i][1] << " & " << mutable_point_for_observations[i + match_num][1] << std::endl;
    //     std::cout << "point_z: " << point3D_arm.at<cv::Vec3f>(i, 0)[2] << " -> " << mutable_point_for_observations[i][2] << " & " << mutable_point_for_observations[i + match_num][2] << std::endl
    //               << std::endl;
    // }

    // publish用データ
    cv::Mat p3_BA;
    for (size_t i = 0; i < 2 * match_num; ++i)
    {
        cv::Point3f p_xyz;
        p_xyz.x = (float)mutable_point_for_observations[i][0];
        p_xyz.y = (float)mutable_point_for_observations[i][1];
        p_xyz.z = (float)mutable_point_for_observations[i][2];
        p3_BA.push_back(p_xyz);
    }
    point3D_BA = p3_BA.clone();

    // delete
    for (size_t i = 0; i < 2 * match_num; i++)
    {
        delete[] mutable_point_for_observations[i];
    }
    delete[] mutable_point_for_observations;
}

void Reconstruction::estimate_move()
{
    // 眼球の移動を検知したらフラグを管理する
    if (!flag_estimate_move)
        return;

    // 5点アルゴリズムやる以上、5点以上の点が必要
    if (matched_point1.size() < 5)
    {
        return;
    }
    // 5点アルゴリズム//１つめの画像を正規化座標としたときに2枚目の画像への回転・並進変換行列
    cv::Mat essential_mask;
    cv::Point2f principle_point(U0, V0);
    cv::Mat EssentialMat = cv::findEssentialMat(matched_point1, matched_point2, (FOCAL_X + FOCAL_Y) / 2., principle_point, cv::RANSAC, 0.9999, 0.003, essential_mask);
    cv::recoverPose(EssentialMat, matched_point1, matched_point2, R_est, t_est, (FOCAL_X + FOCAL_Y) / 2., principle_point, essential_mask);
}

void Reconstruction::showImage()
{
    if (!flag_showImage)
        return;

    // マッチングの様子を図示
    cv::Scalar match_line_color = cv::Scalar(255, 0, 0);
    cv::Scalar match_point_color = cv::Scalar(255, 255, 0);
    cv::drawMatches(frame_data.extractor.image, frame_data.extractor.keypoints,
                    keyframe_data.extractor.image, keyframe_data.extractor.keypoints,
                    inliners_matches, matching_image, match_line_color, match_point_color);

    // カメラのuv方向への移動量を矢印で追加で図示
    cv::Point2f image_center = cv::Point2f(frame_data.extractor.image.rows / 2., frame_data.extractor.image.cols / 2.);
    cv::Point2f image_center2 = cv::Point2f(frame_data.extractor.image.rows * 3. / 2., frame_data.extractor.image.cols / 2.);
    cv::Scalar color_arrow = cv::Scalar(0, 255, 0);
    cv::Point2f center_t_arm = cv::Point2f(frame_data.camerainfo.Transform.at<float>(0) * 20 + image_center.x,
                                           frame_data.camerainfo.Transform.at<float>(1) * 20 + image_center.y);
    cv::Point2f center_t_arm_z = cv::Point2f(image_center2.x,
                                             frame_data.camerainfo.Transform.at<float>(2) * 20 + image_center.y);
    cv::arrowedLine(matching_image, image_center, center_t_arm, color_arrow, 2, 8, 0, 0.5);
    cv::arrowedLine(matching_image, image_center2, center_t_arm_z, color_arrow, 2, 8, 0, 0.5);

    // マッチングの様子なしの比較画像を図示
    cv::Mat left_image = keyframe_data.extractor.image;
    cv::Mat right_image = frame_data.extractor.image;
    cv::Mat nomatching_image;
    cv::hconcat(left_image, right_image, nomatching_image);

    // 表示
    cv::imshow("matching_image", matching_image);
    // cv::imshow("nomatching_image", nomatching_image);
    cv::waitKey(5);
}

void Reconstruction::publish(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
    if (flag_setFirstFrame)
    {
        flag_setFirstFrame = false;
        return;
    }

    // cv::Mat pointCloud(match_num, 1, CV_32FC3);
    // point3D.convertTo(pointCloud, CV_32FC3);
    // auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    // converter.cvMat_to_msgPointCloud2(pointCloud, *msg_cloud_pub, 0);
    // pub_pointcloud->publish(std::move(msg_cloud_pub));

    cv::Mat pointCloud_arm(match_num, 1, CV_32FC3);
    point3D_arm.convertTo(pointCloud_arm, CV_32FC3);
    auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    converter.cvMat_to_msgPointCloud2(pointCloud_arm, *msg_cloud_pub, 0);
    pub_pointcloud->publish(std::move(msg_cloud_pub));

    // cv::Mat pointCloud_BA(match_num, 1, CV_32FC3);
    // point3D_BA.convertTo(pointCloud_BA, CV_32FC3);
    // auto msg_cloud_BA_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    // converter.cvMat_to_msgPointCloud2(pointCloud_BA, *msg_cloud_BA_pub, 0);
    // pub_pointcloud->publish(std::move(msg_cloud_BA_pub));
}

void Reconstruction::process()
{
    flag_reconstruction = true;

    if (flag_setFirstFrame)
    {
        this->setFirstFrame();
        printf("FirstFrame was setted!\n");
        return;
    }

    // どのKFを使うか選択しkeyframe_dataに入力
    this->chooseKeyFrame();

    // 現在のフレームがKeyFrameの候補であれば
    this->setKeyFrame();

    if (!flag_reconstruction)
        return;

    // 特徴点マッチング
    this->BF_matching();

    // 誤対応除去
    this->BF_outlier_remover();

    // マッチング量が5未満は眼球移動量推定できないのでパス
    if (match_num < 5)
        return;

    // もし眼球移動を検知すれば
    this->estimate_move();

    // 三角測量
    this->triangulation();
    // this->triangulation_test();
    // this->triangulation_est();

    // バンドル調整
    // this->bundler();

    // 図示
    this->showImage();
}

void Reconstruction::topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                                     const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
    printf("Received image #%s\n", msg_image->header.frame_id.c_str());
    // 初期化
    this->initialize();
    // Subscribeしたものをframe_dataに入力
    this->input_data(msg_image, msg_arm);
    // メインの処理
    this->process();
    // Publish
    this->publish(pub_pointcloud);
}