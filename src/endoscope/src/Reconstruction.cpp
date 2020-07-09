#include "../include/endoscope/Reconstruction.hpp"
#include "../include/endoscope/triangulate.hpp"
#include "../include/endoscope/Bundler.hpp"
#include "../include/endoscope/cost_function.hpp"
#include "../../HTL/include/transform.h"
#include "../../HTL/include/msg_converter.h"

Transform transform;
Converter converter;

Reconstruction::Reconstruction()
    : flag_reconstruction(false), flag_setFirstFrame(true), flag_showImage(true), flag_estimate_move(false),
      threshold_knn_ratio(0.7f), threshold_ransac(5.0)
{
    if (flag_showImage)
    {
        //ウィンドウの用意
        cv::namedWindow("matching_image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("nomatching_image", cv::WINDOW_AUTOSIZE);
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
            if (abs(t_endo.at<float>(2)) < 0.01)
            {
                // 判定条件2: xy方向の変化or仰角の変化が一定範囲内にある
                // xy方向の移動量
                cv::Point2f t_move_xy(t_endo.at<float>(0), t_endo.at<float>(1));
                bool moving_xy = cv::norm(t_move_xy) < 0.03 && cv::norm(t_move_xy) > 0.005;
                // 仰角
                float phi = transform.RevFromRotMat(itr->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);
                bool moving_phi = abs(phi) < 0.05 && abs(phi) > 0.005;
                if (moving_xy && moving_phi)
                {
                    // printf("あったぞ＾＾(xy: %f, phi: %f)\n", cv::norm(t_move_xy), abs(phi));
                    std::cout << "KeyFrame No." << itr->camerainfo.frame_num << " is used." << std::endl;
                    keyframe_itr = itr;
                    keyframe_data = *keyframe_itr;
                    flag_reconstruction = true;
                    return;
                }
                // printf("ないぞ(xy: %f, phi: %f)\n", cv::norm(t_move_xy), abs(phi));
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
        std::cout << "KeyFrame_Databade size is " << keyframe_database.size() << std::endl;
        flag_reconstruction = false;
        this->setKeyFrame();
        return;
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
    cv::Mat t_move = itr_endo->camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - itr_endo->camerainfo.Transform_world);
    cv::Point2f t_move_xy;
    t_move_xy.x = t_move.at<float>(0);
    t_move_xy.y = t_move.at<float>(1);
    float t_move_norm = cv::norm(t_move_xy);

    // 仰角
    float phi = transform.RevFromRotMat(itr_endo->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);

    if (frame_span > 1 && (t_move_norm > 0.01 || abs(phi) > 0.05))
    {
        // printf("(xy: %f, phi: %f)\n", t_move_norm, abs(phi));
        return true;
    }
    // printf("(xy: %f, phi: %f)\n", t_move_norm, abs(phi));
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
    frame_data.camerainfo.setData();

    // 画像のID情報
    frame_data.camerainfo.frame_num = atoi(msg_image->header.frame_id.c_str());

    // 特徴点検出・特徴量記述
    frame_data.extractor.extractAndcompute(Extractor::DetectorType::AKAZE);

    // コマンドラインに表示
    // std::cout << "frame_id : " << frame_data.camerainfo.frame_num << std::endl;
    // std::cout << "Rotation_world : " << frame_data.camerainfo.Rotation_world << std::endl;
    // std::cout << "Trans_world : " << frame_data.camerainfo.Transform_world << std::endl;
}

void Reconstruction::knn_matching()
{
    if (keyframe_database.empty())
    {
        printf("keyframe_database is empty!\n");
        return;
    }
    // descriptorはCV_32FじゃないとFLANNでのマッチングはできないらしい
    if (frame_data.extractor.descirptors.type() != CV_32F)
        frame_data.extractor.descirptors.convertTo(frame_data.extractor.descirptors, CV_32F);
    if (keyframe_data.extractor.descirptors.type() != CV_32F)
        keyframe_data.extractor.descirptors.convertTo(keyframe_data.extractor.descirptors, CV_32F);
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    matcher->knnMatch(keyframe_data.extractor.descirptors, frame_data.extractor.descirptors, knn_matches, 2);
}

void Reconstruction::BF_matching()
{
    if (keyframe_database.empty())
    {
        printf("keyframe_database is empty!\n");
        return;
    }
    // descriptorはCV_32FじゃないとFLANNでのマッチングはできないらしい
    if (frame_data.extractor.descirptors.type() != CV_32F)
        frame_data.extractor.descirptors.convertTo(frame_data.extractor.descirptors, CV_32F);
    if (keyframe_data.extractor.descirptors.type() != CV_32F)
        keyframe_data.extractor.descirptors.convertTo(keyframe_data.extractor.descirptors, CV_32F);
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    matcher->match(keyframe_data.extractor.descirptors, frame_data.extractor.descirptors, dmatch);
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
            match_point1.push_back(keyframe_data.extractor.keypoints[knn_matches[i][0].queryIdx].pt);
            match_point2.push_back(frame_data.extractor.keypoints[knn_matches[i][0].trainIdx].pt);
            dmatch.push_back(knn_matches[i][0]);
        }
    }

    // 誤対応除去②：ホモグラフィ変換を行うときのRANSACを用いる
    cv::Mat homography, inliner_mask;
    // std::vector<cv::KeyPoint> inliners1_keypoints, inliners2_keypoints;
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
            matched_point1.push_back(keyframe_data.extractor.keypoints[dmatch[i].queryIdx].pt);
            matched_point2.push_back(frame_data.extractor.keypoints[dmatch[i].trainIdx].pt);
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
        match_point1.push_back(frame_data.extractor.keypoints[dmatch[i].trainIdx].pt);
        match_point2.push_back(keyframe_data.extractor.keypoints[dmatch[i].queryIdx].pt);
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
            matched_point1.push_back(frame_data.extractor.keypoints[dmatch[i].trainIdx].pt);
            matched_point2.push_back(keyframe_data.extractor.keypoints[dmatch[i].queryIdx].pt);

            // まだ特徴点辞書にindexの登録がなければkeyframeのぶんもここでいれとく
            if (keyframe_data.keyponit_map.count(dmatch[i].queryIdx) == 0)
            {
                MatchedData matchData_key(keyframe_data.extractor.keypoints[dmatch[i].queryIdx].pt,
                                          keyframe_data.camerainfo.ProjectionMatrix.clone(),
                                          keyframe_data.camerainfo.frame_num);
                keyframe_data.keyponit_map.insert(std::make_pair(dmatch[i].queryIdx, matchData_key));
            }

            // keyframe_dataにマッチングした点のindexをkeyとして辞書(multimap)として登録
            MatchedData matchData(frame_data.extractor.keypoints[dmatch[i].trainIdx].pt,
                                  frame_data.camerainfo.ProjectionMatrix.clone(),
                                  frame_data.camerainfo.frame_num);
            keyframe_data.keyponit_map.insert(std::make_pair(dmatch[i].queryIdx, matchData));
        }
    }
    match_num = inliners_matches.size();

    // keyframe_databaseの中から抽出したkeyframe_dataを変更する
    // keyframe_databaseからkeyframeを一旦削除し、新たに作成したkeyframe_dataを同じ位置を指定して挿入する
    keyframe_database.erase(keyframe_itr);
    FrameDatabase newKeyFrameData = keyframe_data;
    keyframe_database.insert(keyframe_itr, newKeyFrameData);
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
    keyframe_data.extractor.match2point_query(inliners_matches);
    frame_data.extractor.match2point_train(inliners_matches);

    // 三角測量
    cv::Mat p3;
    for (size_t i = 0; i < match_num; i++)
    {
        cv::Mat point4D(4, match_num, CV_32FC1);
        cv::Mat point3D_result;
        cv::triangulatePoints(keyframe_data.camerainfo.ProjectionMatrix, frame_data.camerainfo.ProjectionMatrix,
                              cv::Mat(keyframe_data.extractor.point[i]), cv::Mat(frame_data.extractor.point[i]),
                              point4D);
        cv::convertPointsFromHomogeneous(point4D.reshape(4, 1), point3D_result);
        cv::Mat point3D_mine = Triangulate::triangulation(keyframe_data.extractor.point[i], keyframe_data.camerainfo.ProjectionMatrix,
                                                          frame_data.extractor.point[i], frame_data.camerainfo.ProjectionMatrix);
        // std::cout << "rows, cols, ch: " << point3D_result.rows << ", " << point3D_result.cols << ", " << point3D_result.channels() << std::endl;
        // std::cout << "point3D_mine : " << point3D_mine << std::endl;
        p3.push_back(point3D_mine.reshape(3, 1));
        point3D_hold.push_back(point3D_mine.reshape(3, 1));
    }
    point3D = p3.clone();
}

void Reconstruction::triangulation_multiscene()
{
    cv::Mat p3;
    // printf("keyframe_data.extractor.keypoints.size() = %zu\n", keyframe_data.extractor.keypoints.size());
    // 今回使ったkeyframeがもつ特徴点毎に辞書を作成しているので、特徴点毎に計算
    for (size_t i = 0; i < keyframe_data.extractor.keypoints.size(); i++)
    {
        // マッチング辞書の中で5個以上マッチングframeを発見したものを探索
        if (keyframe_data.keyponit_map.count(i) >= KEYPOINT_SCENE)
        {
            // 3次元復元用データコンテナ
            std::vector<cv::Point2f> point2D;
            std::vector<cv::Mat> ProjectMat;

            typedef std::multimap<unsigned int, MatchedData>::iterator iterator;
            std::pair<iterator, iterator> range = keyframe_data.keyponit_map.equal_range(i);
            for (iterator itr = range.first; itr != range.second; itr++)
            {
                point2D.push_back(itr->second.image_points);
                ProjectMat.push_back(itr->second.ProjectionMatrix);
            }

            // 三次元復元
            cv::Mat point3D_result = Triangulate::triangulation(point2D, ProjectMat);
            // std::cout << "KeyPoint Index : " << i << std::endl;
            // std::cout << "point3D_result : " << std::endl
            //           << point3D_result << std::endl;
            p3.push_back(point3D_result.reshape(3, 1));
            point3D_hold.push_back(point3D_result.reshape(3, 1));

            // 終わったら保持してたものを削除
            keyframe_data.keyponit_map.erase(i);
        }
    }
    point3D = p3.clone();
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
        mutable_point_for_observations[i][0] = (double)point3D.at<cv::Vec3f>(i, 0)[0];
        mutable_point_for_observations[i][1] = (double)point3D.at<cv::Vec3f>(i, 0)[1];
        mutable_point_for_observations[i][2] = (double)point3D.at<cv::Vec3f>(i, 0)[2];
        mutable_point_for_observations[i + match_num][0] = (double)point3D.at<cv::Vec3f>(i, 0)[0];
        mutable_point_for_observations[i + match_num][1] = (double)point3D.at<cv::Vec3f>(i, 0)[1];
        mutable_point_for_observations[i + match_num][2] = (double)point3D.at<cv::Vec3f>(i, 0)[2];

        //コスト関数
        // ceres::CostFunction *cost_function_train = SnavelyReprojectionError::Create((double)keyframe_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.x,
        //                                                                             (double)keyframe_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.y);
        // ceres::CostFunction *cost_function_query = SnavelyReprojectionError::Create((double)frame_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.x,
        //                                                                             (double)frame_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.y);
        // problem.AddResidualBlock(cost_function_train, NULL, mutable_camera_for_observations[0], mutable_point_for_observations[i]);
        // problem.AddResidualBlock(cost_function_query, NULL, mutable_camera_for_observations[1], mutable_point_for_observations[i + match_num]);

        ceres::CostFunction *cost_function_query = ProjectionErrorCostFuctor::Create((double)keyframe_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.x,
                                                                                     (double)keyframe_data.extractor.keypoints[inliners_matches[i].queryIdx].pt.y);
        ceres::CostFunction *cost_function_train = ProjectionErrorCostFuctor::Create((double)frame_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.x,
                                                                                     (double)frame_data.extractor.keypoints[inliners_matches[i].trainIdx].pt.y);
        problem.AddResidualBlock(cost_function_query, NULL,
                                 &mutable_camera_for_observations[0][0], &mutable_camera_for_observations[0][1], &mutable_camera_for_observations[0][2],
                                 &mutable_camera_for_observations[0][3], &mutable_camera_for_observations[0][4], &mutable_camera_for_observations[0][5],
                                 &mutable_point_for_observations[i][0], &mutable_point_for_observations[i][1], &mutable_point_for_observations[i][2]);
        problem.AddResidualBlock(cost_function_train, NULL,
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
    //     std::cout << "point_x: " << point3D.at<cv::Vec3f>(i, 0)[0] << " -> " << mutable_point_for_observations[i][0] << " & " << mutable_point_for_observations[i + match_num][0] << std::endl;
    //     std::cout << "point_y: " << point3D.at<cv::Vec3f>(i, 0)[1] << " -> " << mutable_point_for_observations[i][1] << " & " << mutable_point_for_observations[i + match_num][1] << std::endl;
    //     std::cout << "point_z: " << point3D.at<cv::Vec3f>(i, 0)[2] << " -> " << mutable_point_for_observations[i][2] << " & " << mutable_point_for_observations[i + match_num][2] << std::endl
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
    cv::drawMatches(keyframe_data.extractor.image, keyframe_data.extractor.keypoints,
                    frame_data.extractor.image, frame_data.extractor.keypoints,
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
    cv::imshow("nomatching_image", nomatching_image);
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

    cv::Mat pointCloud_hold(match_num, 1, CV_32FC3);
    point3D_hold.convertTo(pointCloud_hold, CV_32FC3);
    auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    converter.cvMat_to_msgPointCloud2(pointCloud_hold, *msg_cloud_pub, 0);
    pub_pointcloud->publish(std::move(msg_cloud_pub));

    // cv::Mat pointCloud_BA(match_num, 1, CV_32FC3);
    // point3D_BA.convertTo(pointCloud_BA, CV_32FC3);
    // auto msg_cloud_BA_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    // converter.cvMat_to_msgPointCloud2(pointCloud_BA, *msg_cloud_BA_pub, 0);
    // pub_pointcloud->publish(std::move(msg_cloud_BA_pub));
}

void Reconstruction::process()
{
    flag_reconstruction = false;

    if (flag_setFirstFrame)
    {
        this->setFirstFrame();
        printf("FirstFrame was setted!\n");
        return;
    }

    // どのKFを使うか選択しkeyframe_dataに入力
    this->chooseKeyFrame();

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
    // this->estimate_move();

    // 三角測量
    // this->triangulation();
    this->triangulation_multiscene();

    // バンドル調整
    // this->bundler();

    // 図示
    this->showImage();
}

void Reconstruction::topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                                     const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
    printf("\nReceived image #%s\n", msg_image->header.frame_id.c_str());
    // 初期化
    this->initialize();
    // Subscribeしたものをframe_dataに入力
    this->input_data(msg_image, msg_arm);
    // メインの処理
    this->process();
    // Publish
    this->publish(pub_pointcloud);
}