#include "../include/endoscope/Reconstruction.hpp"
#include "../include/endoscope/triangulate.hpp"
#include "../include/endoscope/Bundler.hpp"
#include "../include/endoscope/cost_function.hpp"
#include "../../HTL/include/transform.h"
#include "../../HTL/include/msg_converter.h"

Transform transform;
Converter converter;

Reconstruction::Reconstruction()
    : flag_reconstruction(false), flag_setFirstFrame(true), flag_showImage(false), flag_estimate_move(false),
      threshold_knn_ratio(0.7f), threshold_ransac(5.0),
      num_CPU_core(8), num_Scene(KEYPOINT_SCENE),
      matching_method(Matching::BruteForce), extract_type(Extractor::DetectorType::AKAZE),
      publish_type(Publish::FILTER_HOLD), use_mode(UseMode::NORMAL_SCENE) {}

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
    // 図示
    this->showImage();
    // Publish
    this->publish(pub_pointcloud);
}

void Reconstruction::process()
{
    if (flag_setFirstFrame)
    {
        this->setFirstFrame();
        printf("FirstFrame was setted!\n");
        return;
    }

    // どのKFを使うか選択しkeyframe_dataに入力
    this->chooseKeyFrame();

    // Keyframeならば挿入する
    this->setKeyFrame();

    if (!flag_reconstruction)
        return;

    // カメラの位置姿勢を計算
    this->setCameraInfo();

    // 特徴点マッチングと誤対応除去
    this->matching();
    this->outlier_remover();

    // もし眼球移動を検知すれば
    this->estimate_move();

    if (!flag_reconstruction)
        return;

    // 多視点での三角測量
    this->triangulate(); // 三角測量

    // バンドル調整
    this->bundler(); // バンドル調整  

    // マップの管理を行う（グラフ構造を実装しようかな？）
    // this->manageMap();
}

void Reconstruction::initialize()
{
    // std::vector のclear
    dmatch.clear();
    inliners_matches.clear();
    matched_point1.clear();
    matched_point2.clear();
    frame_data.extractor.point.clear();
    frame_data.extractor.keypoints.clear();
    keyframe_data.extractor.point.clear();
    keyframe_data.extractor.keypoints.clear();

    // std::map のclear
    camerainfo_map.clear();
    pointData_map.clear();
    framenum_cam_map.clear();
    framenum_point_map.clear();

    frame_data.camerainfo.CameraMatrix = this->CameraMat.clone();
    flag_reconstruction = false;
}

void Reconstruction::setFirstFrame()
{
    frame_data.camerainfo.Rotation = Rotation_eye.clone();
    frame_data.camerainfo.Transform = Transform_zeros.clone();
    keyframe_database.push_back(frame_data);
}

// 現在のフレームとマッチングさせるKeyFrameをKeyFrameDatabaseから一つ選ぶ
void Reconstruction::chooseKeyFrame()
{
    // keyframe_databaseの中身が充実してないときは、とにかくkeyframeを挿入するだけ行う
    if (keyframe_database.size() < KEYFRAME_DATABASE_NUM)
    {
        std::cout << "KeyFrame_Database size is " << keyframe_database.size() << std::endl;
        flag_reconstruction = false;
        this->setKeyFrame();
        return;
    }

    float Z_MAX, XY_MIN, XY_MAX, PHI_MIN, PHI_MAX;
    switch (use_mode)
    {
    case UseMode::NORMAL_SCENE:
        Z_MAX = CHOOSE_KF_Z_MAX_N;
        XY_MIN = CHOOSE_KF_XY_MIN_N;
        XY_MAX = CHOOSE_KF_XY_MAX_N;
        PHI_MIN = CHOOSE_KF_PHI_MIN_N;
        PHI_MAX = CHOOSE_KF_PHI_MAX_N;
        break;

    case UseMode::EYE:
        Z_MAX = CHOOSE_KF_Z_MAX_E;
        XY_MIN = CHOOSE_KF_XY_MIN_E;
        XY_MAX = CHOOSE_KF_XY_MAX_E;
        PHI_MIN = CHOOSE_KF_PHI_MIN_E;
        PHI_MAX = CHOOSE_KF_PHI_MAX_E;
        break;
    }

    // 新しく登録したキーフレームから探索する
    // for (auto itr = keyframe_database.end() - 1; itr != keyframe_database.begin() - 1; --itr)
    for (auto itr = keyframe_database.begin(); itr != keyframe_database.end(); ++itr)
    {
        // 判定条件1: Z方向の変化が少ない
        // カメラ座標での移動量の計算
        cv::Mat t_endo = itr->camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - itr->camerainfo.Transform_world);
        bool moving_z_max = std::abs(t_endo.at<float>(2)) < Z_MAX;

        // 判定条件2: xy方向の変化or仰角の変化が一定範囲内にある
        // xy方向の移動量
        cv::Point2f t_move_xy(t_endo.at<float>(0), t_endo.at<float>(1));
        bool moving_xy_max = cv::norm(t_move_xy) < XY_MAX;
        bool moving_xy_min = cv::norm(t_move_xy) > XY_MIN;
        // 仰角
        float phi = transform.RevFromRotMat(itr->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);
        bool moving_phi_max = std::abs(phi) < PHI_MAX;
        bool moving_phi_min = std::abs(phi) > PHI_MIN;

        // printf("KF #%d: t_z = %f, xy = %f, phi = %f (%f)\n", itr->camerainfo.frame_num, std::abs(t_endo.at<float>(2)), cv::norm(t_move_xy), std::abs(phi), std::abs(phi) * 180 / M_PI);
        if (moving_z_max && moving_xy_max && moving_phi_max && (moving_xy_min || moving_phi_min))
        {
            // 見つけた
            std::cout << "KeyFrame No." << itr->camerainfo.frame_num << " is used" << std::endl;
            keyframe_itr = itr;
            keyframe_data = *keyframe_itr;
            flag_reconstruction = true;
            return;
        }
        // printf("戻るぞ\n");
    }
    // 何一つ当てはまるのがなければ三次元復元は行わない
    flag_reconstruction = false;
    printf("なかったよ\n");
    return;
}

void Reconstruction::setKeyFrame()
{
    // keyframeが見つかっているときなので、今回はkeyframeは設定しない
    if (flag_reconstruction)
    {
        return;
    }

    float Z_MAX, XY_MAX, PHI_MAX;
    switch (use_mode)
    {
    case UseMode::NORMAL_SCENE:
        Z_MAX = SET_KF_Z_MAX_N;
        XY_MAX = SET_KF_XY_MAX_N;
        PHI_MAX = SET_KF_PHI_MAX_N;
        break;

    case UseMode::EYE:
        Z_MAX = SET_KF_Z_MAX_E;
        XY_MAX = SET_KF_XY_MAX_E;
        PHI_MAX = SET_KF_PHI_MAX_E;
        break;
    }

    // 新しく登録したキーフレームから探索する(すぐ見つかりやすいので高速になる)
    for (auto itr = keyframe_database.end() - 1; itr != keyframe_database.begin() - 1; --itr)
    {
        // 判定条件1: Z方向の変化が少ない
        // カメラ座標での移動量の計算
        cv::Mat t_endo = itr->camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - itr->camerainfo.Transform_world);
        if (std::abs(t_endo.at<float>(2)) < Z_MAX)
        {
            // 判定条件2: xy方向の変化or仰角の変化が一定範囲内にある
            // xy方向の移動量
            cv::Point2f t_move_xy(t_endo.at<float>(0), t_endo.at<float>(1));
            bool moving_xy = cv::norm(t_move_xy) < XY_MAX;
            // 仰角
            float phi = transform.RevFromRotMat(itr->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);
            bool moving_phi = std::abs(phi) < PHI_MAX;
            if (moving_xy && moving_phi)
            {
                return;
            }
        }
    }
    // 何一つ当てはまるのがなければKFとする
    frame_data.camerainfo.Rotation = Rotation_eye.clone();
    frame_data.camerainfo.Transform = Transform_zeros.clone();
    keyframe_database.push_back(frame_data);
    printf("KeyFrame was setted!\n");
    return;
}

void Reconstruction::setCameraInfo()
{
    // カメラ座標の計算
    frame_data.camerainfo.Rotation = keyframe_data.camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world;
    frame_data.camerainfo.Transform = keyframe_data.camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - keyframe_data.camerainfo.Transform_world);
    keyframe_data.camerainfo.Rotation = Rotation_eye.clone();
    keyframe_data.camerainfo.Transform = Transform_zeros.clone();
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
    frame_data.extractor.extractAndcompute(extract_type);

    // コマンドラインに表示
    // std::cout << "frame_id : " << frame_data.camerainfo.frame_num << std::endl;
    // std::cout << "Rotation_world : " << frame_data.camerainfo.Rotation_world << std::endl;
    // std::cout << "Trans_world : " << frame_data.camerainfo.Transform_world << std::endl;
}

void Reconstruction::matching()
{
    // 特徴点マッチングと誤対応除去
    switch (matching_method)
    {
    case KNN:
        this->knn_matching();
        break;

    case BruteForce:
        this->BF_matching();
        break;

    default:
        std::cout << "--match option is not correct" << std::endl;
        return;
        break;
    }
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

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(keyframe_data.extractor.descirptors, frame_data.extractor.descirptors, knn_matches, 2);

    // 誤対応除去：knnマッチングでなるべく差が大きいものだけを選択(Brute-Forceではこれは不必要)
    // 距離が小さいほうがよりマッチング度合いが高い
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < threshold_knn_ratio * knn_matches[i][1].distance)
        {
            dmatch.push_back(knn_matches[i][0]);
        }
    }
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

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    matcher->match(keyframe_data.extractor.descirptors, frame_data.extractor.descirptors, dmatch);
}

void Reconstruction::outlier_remover()
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

    // 誤対応除去１
    // ホモグラフィ変換を行うときのRANSACを用いる
    std::vector<int> inliners_idx;
    cv::Mat homography, inliner_mask;
    std::vector<cv::KeyPoint> inliners1_keypoints, inliners2_keypoints;
    inliners_idx.clear();
    homography = cv::findHomography(match_point1, match_point2, inliner_mask, cv::RANSAC, threshold_ransac);

    std::vector<cv::Point2f> distance;
    for (int i = 0; i < inliner_mask.rows; i++)
    {
        if (inliner_mask.at<uchar>(i))
        {
            inliners_idx.push_back(i); // インデックスを整理した後にもオリジナルのインデックスを参照できるように保存

            // 重ね合わせたときの距離を計算
            cv::Point2f keyframe_pt = keyframe_data.extractor.keypoints[dmatch[i].queryIdx].pt;
            cv::Point2f frame_pt = frame_data.extractor.keypoints[dmatch[i].trainIdx].pt;
            cv::Point2f vec;
            vec.x = frame_pt.x - keyframe_pt.x;
            vec.y = frame_pt.y - keyframe_pt.y;
            distance.push_back(vec);
        }
    }

    // 誤対応除去２
    // 画像を重ね合わせたときの対応する点の座標間の距離を計算し、その距離の平均と分散を用いて範囲外の対応を削除
    // 平均
    cv::Point2f distance_average;
    for (auto itr = distance.begin(); itr != distance.end(); itr++)
    {
        distance_average.x += itr->x;
        distance_average.y += itr->y;
    }
    distance_average.x /= distance.size();
    distance_average.y /= distance.size();

    // 分散
    cv::Point2f distance_variance;
    for (auto itr = distance.begin(); itr != distance.end(); itr++)
    {
        distance_variance.x += (itr->x - distance_average.x) * (itr->x - distance_average.x);
        distance_variance.y += (itr->y - distance_average.y) * (itr->y - distance_average.y);
    }
    distance_variance.x /= distance.size();
    distance_variance.y /= distance.size();
    // printf("mean:[%f %f], var:[%f %f]\n", distance_average.x, distance_average.y, distance_variance.x, distance_variance.y);
    if (distance_variance.x > THRESH_VARIANCE || distance_variance.y > THRESH_VARIANCE)
    {
        return;
    }

    // スミルノフ･グラブス検定
    //（外れ値 － 平均値） / σ
    std::vector<int> dmatch_num; // 検定で合格したもののindexが入ってるコンテナ
    for (size_t i = 0; i < distance.size(); i++)
    {
        cv::Point2f smi_grub;
        smi_grub.x = std::fabs(distance[i].x - distance_average.x) / distance_variance.x;
        smi_grub.y = std::fabs(distance[i].y - distance_average.y) / distance_variance.y;
        if (smi_grub.x < THRESH_SMIROFF_GRUBBS && smi_grub.y < THRESH_SMIROFF_GRUBBS)
        {
            dmatch_num.push_back(inliners_idx[i]);
        }
        // printf("distance[%zu] = [%f %f], 検定 = [%f, %f]\n", i, distance[i].x, distance[i].y, smi_grub.x, smi_grub.y);
    }

    // 誤対応除去したものを保存
    // printf("size : [ %zu, %zu ]\n", inliners_idx.size(), dmatch_num.size());
    for (size_t i = 0; i < dmatch_num.size(); i++)
    {
        int num = dmatch_num[i];
        inliners_matches.push_back(dmatch[num]);
        matched_point1.push_back(keyframe_data.extractor.keypoints[dmatch[num].queryIdx].pt);
        matched_point2.push_back(frame_data.extractor.keypoints[dmatch[num].trainIdx].pt);

        // まだ特徴点辞書にindexの登録がなければkeyframeのぶんもここでいれとく
        if (keyframe_data.keyponit_map.count(dmatch[num].queryIdx) == 0)
        {
            MatchedData matchData_key(keyframe_data.extractor.keypoints[dmatch[num].queryIdx].pt,
                                      keyframe_data.camerainfo.ProjectionMatrix.clone(),
                                      keyframe_data.camerainfo.Rotation_world.clone(),
                                      keyframe_data.camerainfo.Transform_world.clone(),
                                      keyframe_data.camerainfo.RodriguesVec_world.clone(),
                                      keyframe_data.camerainfo.frame_num);
            keyframe_data.keyponit_map.insert(std::make_pair(dmatch[num].queryIdx, matchData_key));
        }

        // keyframe_dataにマッチングした点のindexをkeyとして辞書(multimap)として登録
        MatchedData matchData(frame_data.extractor.keypoints[dmatch[num].trainIdx].pt,
                              frame_data.camerainfo.ProjectionMatrix.clone(),
                              frame_data.camerainfo.Rotation_world.clone(),
                              frame_data.camerainfo.Transform_world.clone(),
                              frame_data.camerainfo.RodriguesVec_world.clone(),
                              frame_data.camerainfo.frame_num);
        keyframe_data.keyponit_map.insert(std::make_pair(dmatch[num].queryIdx, matchData));
    }
    match_num = inliners_matches.size();

    // keyframe_databaseの中から抽出したkeyframe_dataを変更する
    // keyframe_databaseからkeyframeを一旦削除し、新たに作成したkeyframe_dataを同じ位置を指定して挿入する
    keyframe_database.erase(keyframe_itr);
    FrameDatabase newKeyFrameData = keyframe_data;
    keyframe_database.insert(keyframe_itr, newKeyFrameData);
}

void Reconstruction::triangulate()
{
    if (!flag_reconstruction)
        return;

    cv::Mat p3;
    // 今回使ったkeyframeがもつ特徴点毎に辞書を作成しているので、特徴点毎に計算
    for (auto dmatch_itr = inliners_matches.begin(); dmatch_itr < inliners_matches.end(); dmatch_itr++)
    {
        // マッチング辞書の中で十分マッチングframeを発見したものを探索
        if (keyframe_data.keyponit_map.count(dmatch_itr->queryIdx) >= num_Scene)
        {
            // 3次元復元用データコンテナ
            std::vector<cv::Point2f> point2D;
            std::vector<cv::Mat> ProjectMat;

            typedef std::multimap<int, MatchedData>::iterator iterator;
            std::pair<iterator, iterator> range = keyframe_data.keyponit_map.equal_range(dmatch_itr->queryIdx);
            for (iterator itr = range.first; itr != range.second; itr++)
            {
                point2D.push_back(itr->second.image_points);
                ProjectMat.push_back(itr->second.ProjectionMatrix);
            }

            // 三次元復元
            cv::Mat point3D_result = Triangulate::triangulation(point2D, ProjectMat);

            // 点の登録
            p3.push_back(point3D_result.reshape(3, 1));
            point3D_hold.push_back(point3D_result.reshape(3, 1));

            // バンドル調整用データ
            typedef std::multimap<int, MatchedData>::iterator iterator;
            std::pair<iterator, iterator> range2 = keyframe_data.keyponit_map.equal_range(dmatch_itr->queryIdx);
            for (iterator itr = range2.first; itr != range2.second; itr++)
            {
                CameraInfo temp_camerainfo;
                temp_camerainfo.ProjectionMatrix = itr->second.ProjectionMatrix.clone();
                temp_camerainfo.Rotation_world = itr->second.Rotation_world.clone();
                temp_camerainfo.Transform_world = itr->second.Transform_world.clone();
                temp_camerainfo.RodriguesVec_world = itr->second.RodriguesVec_world.clone();
                camerainfo_map.insert(std::make_pair(itr->second.frame_num, temp_camerainfo));

                PointData temp_pointData;
                temp_pointData.point3D = point3D_result.clone();
                temp_pointData.point2D = itr->second.image_points;
                pointData_map.insert(std::make_pair(itr->second.frame_num, temp_pointData));
            }

            // 終わったら辞書に登録してたフレーム情報を削除
            if (keyframe_data.keyponit_map.count(dmatch_itr->queryIdx) > KEYPOINT_SCENE_DELETE)
                keyframe_data.keyponit_map.erase(dmatch_itr->queryIdx);
        }
    }
    if (!p3.empty())
    {
        point3D = p3.clone();

        // 点群の統計フィルタ
        cv::Mat p3_filter;
        if (this->pointcloud_statics_filter(p3, &p3_filter))
        {
            point3D_filtered = p3_filter.clone();
            point3D_filtered_hold.push_back(point3D_filtered);
        }
    }
}

void Reconstruction::bundler()
{
    if(camerainfo_map.empty())
        return;

    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;

    // カメラ情報
    double **mutable_camera_for_observations = new double *[camerainfo_map.size()];
    for (size_t i = 0; i < camerainfo_map.size(); i++)
    {
        mutable_camera_for_observations[i] = new double[6];
    }

    int i_cam = 0;
    for(auto itr = camerainfo_map.begin(); itr != camerainfo_map.end(); itr++)
    {
        // std::cout << "camerainfo_map Rodrigues : " << itr->first << std::endl
        //         << itr->second.RodriguesVec_world << std::endl;
        // std::cout << "camerainfo_map trnsform : "  << itr->first << std::endl
        //         << itr->second.Transform_world << std::endl;
        mutable_camera_for_observations[i_cam][0] = itr->second.RodriguesVec_world.at<float>(0);
        mutable_camera_for_observations[i_cam][1] = itr->second.RodriguesVec_world.at<float>(1);
        mutable_camera_for_observations[i_cam][2] = itr->second.RodriguesVec_world.at<float>(2);
        mutable_camera_for_observations[i_cam][3] = itr->second.Transform_world.at<float>(0);
        mutable_camera_for_observations[i_cam][4] = itr->second.Transform_world.at<float>(1);
        mutable_camera_for_observations[i_cam][5] = itr->second.Transform_world.at<float>(2);

        framenum_cam_map.insert(std::make_pair(itr->first, i_cam));
        i_cam++;
    }

    // 3次元点情報
    double **mutable_point_for_observations = new double *[pointData_map.size()];
    double **point2d = new double *[pointData_map.size()];
    for (size_t i = 0; i < pointData_map.size(); i++)
    {
        mutable_point_for_observations[i] = new double[3];
        point2d[i] = new double[2];
    }

    int i_point = 0;
    for(auto itr = pointData_map.begin(); itr != pointData_map.end(); itr++)
    {
        // std::cout << "pointData_map point3D : " << itr->first << std::endl
        //           << itr->second.point3D << std::endl;
        mutable_point_for_observations[i_point][0] = itr->second.point3D.at<float>(0);
        mutable_point_for_observations[i_point][1] = itr->second.point3D.at<float>(1);
        mutable_point_for_observations[i_point][2] = itr->second.point3D.at<float>(2);
        point2d[i_point][0] = itr->second.point2D.x;
        point2d[i_point][1] = itr->second.point2D.y;

        framenum_point_map.insert(std::make_pair(itr->first, i_point));
        i_point++;
    }

    // コスト関数に追加
    for(auto frame_itr = framenum_cam_map.begin(); frame_itr != framenum_cam_map.end(); frame_itr++)
    {
        int access_num_cam = framenum_cam_map.at(frame_itr->first);

        typedef std::multimap<int, int>::iterator iterator;
        std::pair<iterator, iterator> range = framenum_point_map.equal_range(frame_itr->first);
        for (iterator itr = range.first; itr != range.second; itr++)
        {
            ceres::CostFunction *cost_function = ProjectionErrorCostFuctor::Create(point2d[itr->second][0],
                                                                                   point2d[itr->second][1]);
            problem.AddResidualBlock(cost_function, NULL,
                        &mutable_camera_for_observations[access_num_cam][0], &mutable_camera_for_observations[access_num_cam][1], &mutable_camera_for_observations[access_num_cam][2],
                        &mutable_camera_for_observations[access_num_cam][3], &mutable_camera_for_observations[access_num_cam][4], &mutable_camera_for_observations[access_num_cam][5],
                        &mutable_point_for_observations[itr->second][0], &mutable_point_for_observations[itr->second][1], &mutable_point_for_observations[itr->second][2]);
        }

    }

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = num_CPU_core;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}


bool Reconstruction::pointcloud_statics_filter(const cv::Mat &Point3D, cv::Mat *output_point3D)
{
    int point_num = Point3D.rows;
    if (point_num == 0)
        return false;

    // 平均
    cv::Point3f point_average;
    for (int i = 0; i < point_num; i++)
    {
        point_average.x += Point3D.at<cv::Vec3f>(i, 0)[0];
        point_average.y += Point3D.at<cv::Vec3f>(i, 0)[1];
        point_average.z += Point3D.at<cv::Vec3f>(i, 0)[2];
    }
    point_average.x /= point_num;
    point_average.y /= point_num;
    point_average.z /= point_num;
    // 分散
    cv::Point3f point_variance;
    for (int i = 0; i < point_num; i++)
    {
        point_variance.x += (Point3D.at<cv::Vec3f>(i, 0)[0] - point_average.x) * (Point3D.at<cv::Vec3f>(i, 0)[0] - point_average.x);
        point_variance.y += (Point3D.at<cv::Vec3f>(i, 0)[1] - point_average.y) * (Point3D.at<cv::Vec3f>(i, 0)[1] - point_average.y);
        point_variance.z += (Point3D.at<cv::Vec3f>(i, 0)[2] - point_average.z) * (Point3D.at<cv::Vec3f>(i, 0)[2] - point_average.z);
    }
    point_average.x /= point_num;
    point_average.y /= point_num;
    point_average.z /= point_num;
    // printf("mean:[%f %f %f], var:[%f %f %f]\n", point_average.x, point_average.y, point_average.z, point_variance.x, point_variance.y, point_variance.z);

    //分散がでかすぎたらアウト
    if (point_variance.x > THRESH_VARIANCE_POINT || point_variance.y > THRESH_VARIANCE_POINT || point_variance.z > THRESH_VARIANCE_POINT)
        return false;

    *output_point3D = Point3D.clone();
    return true;
}

void Reconstruction::estimate_move()
{
    // 眼球の移動を検知したらフラグを管理する
    if (!flag_estimate_move)
        return;

    // 5点アルゴリズムやる以上、5点以上の点が必要
    if (matched_point1.size() < 5)
        return;

    std::cout << "Estimating Eye Moving" << std::endl;

    // 画像座標から正規化カメラ座標系に変換
    // focalについては無視（よくわからんけどこれで動くのでヨシ！）
    std::vector<cv::Point2f> pt1, pt2;
    for (size_t i = 0; i < matched_point1.size(); i++)
    {
        cv::Point2f point1, point2;
        point1.x = matched_point1[i].x - U0;
        point1.y = matched_point1[i].y - V0;
        point2.x = matched_point2[i].x - U0;
        point2.y = matched_point2[i].y - V0;
        pt1.push_back(point1);
        pt2.push_back(point2);
    }

    // 5点アルゴリズム
    // １つめの画像を正規化座標としたときに2枚目の画像への回転・並進変換行列
    cv::Mat R_est_output, t_est_output, mask;
    cv::Point2d principle_point(U0, V0);
    cv::Mat EssentialMat = cv::findEssentialMat(pt1, pt2, 1, cv::Point2f(0, 0), cv::RANSAC, 0.99, 1, mask);
    if(!(EssentialMat.rows == 3 && EssentialMat.cols == 3))
    {
        printf("EssentialMat.rows = %d, EssentialMat.cols = %d\n", EssentialMat.rows, EssentialMat.cols);
        return;
    }
    cv::recoverPose(EssentialMat, pt1, pt2, R_est_output, t_est_output, 1, cv::Point2f(0, 0), mask);

    if (R_est_output.at<double>(0, 0) < 0.8 || R_est_output.at<double>(1, 1) < 0.8 || R_est_output.at<double>(2, 2) < 0.8)
    {
        std::cout << "Estimated, but output is incorrect estimation." << std::endl;
        return;
    }

    // カメラ座標系での移動量推定
    cv::Mat R_est, t_est;
    float abs = (float)cv::norm(frame_data.camerainfo.Transform, cv::NORM_L2);
    R_est = R_est_output.t();
    t_est = -1 * R_est_output.t() * (t_est_output * abs); // t_est_outputを運動学で求めたabsで割っているのが問題点
    
    // frame_dataに推定結果を格納
    R_est.convertTo(Rot_est, CV_32FC1);
    t_est.convertTo(trans_est, CV_32FC1);
    frame_data.camerainfo.Rotation_est = Rot_est.clone();
    frame_data.camerainfo.Transform_est = trans_est.clone();
    frame_data.camerainfo.Rotation_world_est = keyframe_data.camerainfo.Rotation_world * Rot_est;
    frame_data.camerainfo.Transform_world_est = keyframe_data.camerainfo.Rotation_world * trans_est + keyframe_data.camerainfo.Transform_world;
    frame_data.camerainfo.setData_est();

    // 眼球移動量を推定する
    R_eye_move = frame_data.camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world_est;
    t_eye_move = frame_data.camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world_est - frame_data.camerainfo.Transform_world);

    // 運動学での移動方向ベクトルと5点アルゴリズムでの移動方向ベクトルの内積を求める
    float dot_est = (frame_data.camerainfo.Transform / abs).dot(frame_data.camerainfo.Transform_est / abs);
    if(dot_est < THRESH_DOT)
    {
        flag_reconstruction = false;
    }

    // RANSACの結果ハズレ値となったものを除外
    std::vector<cv::Point2f> inline_pt1, inline_pt2;
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<bool>(i))
        {
            keyframe_data.extractor.inline_point.push_back(pt1[i]);
            frame_data.extractor.inline_point.push_back(pt2[i]);
        }
    }

    // for (size_t i = 0; i < pt1.size(); i++)
    // {
    //     std::cout << "p1 : " << pt1[i] << "  p2 : " << pt2[i] << " mask : " << mask.at<bool>(i) << std::endl;
    // }
    // std::cout << "5点アルゴリズム R_est : " << std::endl
    //           << Rot_est << std::endl
    //           << "5点アルゴリズム t_est : " << std::endl
    //           << trans_est << std::endl
    //           << "運動学 R : " << std::endl
    //           << frame_data.camerainfo.Rotation << std::endl
    //           << "運動学 t :" << std::endl
    //           << frame_data.camerainfo.Transform << std::endl;

    // std::cout << "眼球移動量推定 R_eye_move" << std::endl
    //           << R_eye_move << std::endl
    //           << "眼球移動量推定 t_eye_move" << std::endl
    //           << t_eye_move << std::endl;

    // std::cout << "内積 : " << dot_est << std::endl;
}

void Reconstruction::manageMap()
{
    // 三次元点についてマップ上に登録
    this->registMap(point3D);
    // マップ上の点が現在のフレームに存在するかチェックし、なければその点を削除する
    this->checkMapPoint();
}

void Reconstruction::registMap(const cv::Mat &point3D_)
{
    if(point3D_.empty())
        return;

    for(size_t i = 0; i < match_num; i++)
    {
        Map map;
        map.desciptors = frame_data.extractor.descirptors;
        map.point_3D = point3D_.at<cv::Vec3f>(i);
        map_point.push_back(map);
    }
}

void Reconstruction::checkMapPoint()
{
    for(auto itr = map_point.begin(); itr < map_point.end(); itr++)
    { 
        // 三次元点を現frameに再投影を行う
        cv::Mat cam_point(3, 1, CV_32FC1), world_point(4, 1, CV_32FC1);
        world_point.at<float>(0) = itr->point_3D.x;
        world_point.at<float>(1) = itr->point_3D.y;
        world_point.at<float>(2) = itr->point_3D.z;
        world_point.at<float>(3) = 1.0;
        cam_point = frame_data.camerainfo.ProjectionMatrix * world_point;

        cv::Point2f img_point;
        img_point.x = cam_point.at<float>(0) / cam_point.at<float>(2);
        img_point.y = cam_point.at<float>(1) / cam_point.at<float>(2);

        // 再投影点が画像上にそもそもあるか判定
        if(img_point.x < 0 || img_point.y < 0 || img_point.x > IMAGE_WIDTH || img_point.y > IMAGE_HIGHT)
        {
            return;
        }
        
        // 再投影点付近に同等の特徴点があるか探索
        // KeyFrame毎に特徴点を管理してあげるのも手
        // 特徴点を繋ぎ合わせることができるならOK(knn_matchingとおなじ感じでいいんじゃないかな)
    }
}

void Reconstruction::showImage()
{
    if (!flag_showImage)
        return;

    if(keyframe_data.extractor.image.empty())
        return;

    if (!inliners_matches.empty())
    {
        // マッチングの様子を図示
        cv::Scalar match_line_color = cv::Scalar(255, 0, 0);
        cv::Scalar match_point_color = cv::Scalar(255, 255, 0);
        cv::drawMatches(keyframe_data.extractor.image, keyframe_data.extractor.keypoints,
                        frame_data.extractor.image, frame_data.extractor.keypoints,
                        inliners_matches, matching_image, match_line_color, match_point_color);

        // カメラのuv方向への移動量を矢印で追加で図示
        cv::Point2f image_center = cv::Point2f(frame_data.extractor.image.rows / 2., frame_data.extractor.image.cols / 2.);
        cv::Scalar color_arrow = cv::Scalar(0, 0, 255);
        cv::Point2f center_t_arm = cv::Point2f(frame_data.camerainfo.Transform.at<float>(0) * 10000 + image_center.x,
                                               frame_data.camerainfo.Transform.at<float>(1) * 10000 + image_center.y);
        cv::arrowedLine(matching_image, image_center, center_t_arm, color_arrow, 2, 8, 0, 0.5);
        if(!t_eye_move.empty())
        {
            cv::Point2f image_center2 = cv::Point2f(frame_data.extractor.image.rows * 3. / 2., frame_data.extractor.image.cols / 2.);
            cv::Point2f center_t_arm_est = cv::Point2f(trans_est.at<float>(0) * 10000 + image_center2.x,
                                                        trans_est.at<float>(1) * 10000 + image_center2.y);
            cv::arrowedLine(matching_image, image_center2, center_t_arm_est, color_arrow, 2, 8, 0, 0.5);
        }
    }

    // マッチングの様子なしの比較画像を図示
    cv::Mat left_image = keyframe_data.extractor.image.clone();
    cv::Mat right_image = frame_data.extractor.image.clone();
    cv::hconcat(left_image, right_image, nomatching_image);


    // カメラのuv方向への移動量を矢印で追加で図示
    cv::Point2f image_center = cv::Point2f(frame_data.extractor.image.rows / 2., frame_data.extractor.image.cols / 2.);
    cv::Scalar color_arrow = cv::Scalar(0, 0, 255);
    cv::Point2f center_t_arm = cv::Point2f(frame_data.camerainfo.Transform.at<float>(0) * 10000 + image_center.x,
                                            frame_data.camerainfo.Transform.at<float>(1) * 10000 + image_center.y);
    cv::arrowedLine(nomatching_image, image_center, center_t_arm, color_arrow, 2, 8, 0, 0.5);

    // 表示
    if (!matching_image.empty())
    {
        cv::imshow("matching_image", matching_image);
        cv::waitKey(1);
    }
    if (!nomatching_image.empty())
    {
        cv::imshow("nomatching_image", nomatching_image);
        cv::waitKey(1);
    }
}

void Reconstruction::publish(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
    if (flag_setFirstFrame)
    {
        flag_setFirstFrame = false;
        return;
    }
    cv::Mat pointCloud;
    switch (publish_type)
    {
    case Publish::NORMAL:
        point3D.convertTo(pointCloud, CV_32FC3);
        break;
    case Publish::NORMAL_HOLD:
        point3D_hold.convertTo(pointCloud, CV_32FC3);
        break;
    case Publish::BUNDLE:
        point3D_BA.convertTo(pointCloud, CV_32FC3);
        break;
    case Publish::BUNDLE_HOLD:
        point3D_BA_hold.convertTo(pointCloud, CV_32FC3);
        break;
    case Publish::FILTER:
        point3D_filtered.convertTo(pointCloud, CV_32FC3);
        break;
    case Publish::FILTER_HOLD:
        point3D_filtered_hold.convertTo(pointCloud, CV_32FC3);
        break;
    case Publish::ESTIMATE:
        point3D_est.convertTo(pointCloud, CV_32FC3);
        break;
    case Publish::ESTIMATE_HOLD:
        point3D_est_hold.convertTo(pointCloud, CV_32FC3);
        break;
    }

    auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    converter.cvMat_to_msgPointCloud2(pointCloud, *msg_cloud_pub, 0);
    pub_pointcloud->publish(std::move(msg_cloud_pub));
}

void Reconstruction::setThreshold_knn_ratio(float thresh)
{
    this->threshold_knn_ratio = thresh;
}

void Reconstruction::setThreshold_ransac(float thresh)
{
    this->threshold_ransac = thresh;
}

void Reconstruction::setFlagShowImage(bool flag)
{
    this->flag_showImage = flag;
    if (flag_showImage)
    {
        //ウィンドウの用意
        cv::namedWindow("matching_image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("nomatching_image", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("matching_image", 0, 0);
        cv::moveWindow("nomatching_image", 750, 0);
    }
}

void Reconstruction::setFlagEstimationMovement(bool flag)
{
    this->flag_estimate_move = flag;
}

void Reconstruction::setCPUCoreforBundler(int num)
{
    this->num_CPU_core = num;
}

void Reconstruction::setSceneNum(size_t num)
{
    this->num_Scene = num;
}

void Reconstruction::setPublishType(size_t num)
{
    this->publish_type = num;
}

void Reconstruction::setUseMode(size_t num)
{
    this->use_mode = num;
}

void Reconstruction::setMatchingMethod(size_t num)
{
    this->matching_method = num;
}

void Reconstruction::setExtractor(size_t num)
{
    this->extract_type = num;
}