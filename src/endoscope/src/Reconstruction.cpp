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
      matching(Matching::BruteForce), extract_type(Extractor::DetectorType::AKAZE),
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
    switch (matching)
    {
    case KNN:
        this->knn_matching();
        this->knn_outlier_remover();
        break;

    case BruteForce:
        this->BF_matching();
        this->BF_outlier_remover();
        break;

    default:
        std::cout << "-match is not correct" << std::endl;
        return;
        break;
    }

    // もし眼球移動を検知すれば
    this->estimate_move();

    switch (num_Scene)
    {
    case 2:
        // 三角測量
        this->triangulation();
        // バンドル調整
        this->bundler();
        break;

    default:
        // 三角測量とバンドル調整
        this->triangulation_multiscene(); //バンドル調整機能付き
        break;
    }

    // マップの管理を行う（グラフ構造を実装しようかな？）
    // this->manageMap();
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
        if (std::abs(t_endo.at<float>(2)) < Z_MAX && moving_xy_max && moving_phi_max && (moving_xy_min || moving_phi_min))
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
    frame_data.camerainfo.setData();
    keyframe_data.camerainfo.setData();
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
    // 誤対応除去①：knnマッチングでなるべく差が大きいものだけを選択(Brute-Forceではこれは不必要)
    // 距離が小さいほうがよりマッチング度合いが高い
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
    if (dmatch.size() < 5)
    {
        printf("dmatch.size() == %zu\n", dmatch.size());
        return;
    }
    cv::Mat homography, inliner_mask;
    // std::vector<cv::KeyPoint> inliners1_keypoints, inliners2_keypoints;
    std::vector<int> inliners_idx;
    homography = cv::findHomography(match_point1, match_point2, cv::RANSAC, threshold_ransac, inliner_mask);

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

    // 誤対応除去③：分散とスミルノフ･グラブス検定
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

    //分散がでかすぎたらアウト
    if (distance_variance.x > THRESH_VARIANCE || distance_variance.y > THRESH_VARIANCE)
    {
        return;
    }

    // スミルノフ･グラブス検定（外れ値 － 平均値） / σ
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
                                      keyframe_data.camerainfo.frame_num);
            keyframe_data.keyponit_map.insert(std::make_pair(dmatch[num].queryIdx, matchData_key));
        }

        // keyframe_dataにマッチングした点のindexをkeyとして辞書(multimap)として登録
        MatchedData matchData(frame_data.extractor.keypoints[dmatch[num].trainIdx].pt,
                              frame_data.camerainfo.ProjectionMatrix.clone(),
                              frame_data.camerainfo.Rotation_world.clone(),
                              frame_data.camerainfo.Transform_world.clone(),
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
                                      keyframe_data.camerainfo.frame_num);
            keyframe_data.keyponit_map.insert(std::make_pair(dmatch[num].queryIdx, matchData_key));
        }

        // keyframe_dataにマッチングした点のindexをkeyとして辞書(multimap)として登録
        MatchedData matchData(frame_data.extractor.keypoints[dmatch[num].trainIdx].pt,
                              frame_data.camerainfo.ProjectionMatrix.clone(),
                              frame_data.camerainfo.Rotation_world.clone(),
                              frame_data.camerainfo.Transform_world.clone(),
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

void Reconstruction::triangulation()
{
    if (!flag_reconstruction)
        return;

    // マッチング情報を挿入
    keyframe_data.extractor.match2point_query(inliners_matches);
    frame_data.extractor.match2point_train(inliners_matches);

    // 三角測量
    cv::Mat p3;
    for (size_t i = 0; i < match_num; i++)
    {
        cv::Mat point3D_result = Triangulate::triangulation(keyframe_data.extractor.point[i], keyframe_data.camerainfo.ProjectionMatrix,
                                                            frame_data.extractor.point[i], frame_data.camerainfo.ProjectionMatrix);
        p3.push_back(point3D_result.reshape(3, 1));
        point3D_hold.push_back(point3D_result.reshape(3, 1));
    }
    if (!p3.empty())
        point3D = p3.clone();

    // 移動量推定をしているなら別途三次元復元をこちらでも行う
    if (flag_estimate_move)
    {
        if(frame_data.camerainfo.ProjectionMatrix_est.empty())
            return; 
            
        // 三角測量
        cv::Mat p3_est;
        for (size_t i = 0; i < match_num; i++)
        {
            cv::Mat point3D_result = Triangulate::triangulation(keyframe_data.extractor.inline_point[i], keyframe_data.camerainfo.ProjectionMatrix_est,
                                                                frame_data.extractor.inline_point[i], frame_data.camerainfo.ProjectionMatrix_est);
            p3_est.push_back(point3D_result.reshape(3, 1));
            point3D_est_hold.push_back(point3D_result.reshape(3, 1));
        }
        if (!p3_est.empty())
            point3D_est = p3_est.clone();
    }
}

void Reconstruction::triangulation_multiscene()
{
    if (!flag_reconstruction)
        return;

    cv::Mat p3, p3_BA;
    // 今回使ったkeyframeがもつ特徴点毎に辞書を作成しているので、特徴点毎に計算
    for (size_t i = 0; i < keyframe_data.extractor.keypoints.size(); i++)
    {
        // マッチング辞書の中で十分マッチングframeを発見したものを探索
        if (keyframe_data.keyponit_map.count(i) >= num_Scene)
        {
            // 3次元復元用データコンテナ
            std::vector<cv::Point2f> point2D;
            std::vector<cv::Mat> ProjectMat;
            std::vector<MatchedData> matchdata;

            typedef std::multimap<unsigned int, MatchedData>::iterator iterator;
            std::pair<iterator, iterator> range = keyframe_data.keyponit_map.equal_range(i);
            for (iterator itr = range.first; itr != range.second; itr++)
            {
                point2D.push_back(itr->second.image_points);
                ProjectMat.push_back(itr->second.ProjectionMatrix);
                matchdata.push_back(itr->second);
            }

            // 三次元復元
            cv::Mat point3D_result = Triangulate::triangulation(point2D, ProjectMat);

            // バンドル調整
            cv::Mat point3D_bundler = this->bundler_multiscene(matchdata, point3D_result);

            // 点の登録
            p3.push_back(point3D_result.reshape(3, 1));
            point3D_hold.push_back(point3D_result.reshape(3, 1));
            p3_BA.push_back(point3D_bundler.reshape(3, 1));
            point3D_BA_hold.push_back(point3D_bundler.reshape(3, 1));

            // 終わったら辞書に登録してたフレーム情報を削除
            if (keyframe_data.keyponit_map.count(i) > KEYPOINT_SCENE_DELETE)
                keyframe_data.keyponit_map.erase(i);
        }
    }
    if (!p3.empty())
    {
        point3D = p3.clone();
        point3D_BA = p3_BA.clone();

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

    // 点群の統計フィルタ
    cv::Mat p3_filter;
    if (this->pointcloud_statics_filter(p3_BA, &p3_filter))
    {
        point3D_filtered = p3_filter.clone();
        point3D_filtered_hold.push_back(point3D_filtered);
    }
}

cv::Mat Reconstruction::bundler_multiscene(const std::vector<MatchedData> &matchdata,
                                           const cv::Mat &Point3D)
{
    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;

    //バンドル調整用パラメータ
    size_t size = matchdata.size();
    double **mutable_camera_for_observations = new double *[size];
    for (size_t i = 0; i < size; i++)
    {
        mutable_camera_for_observations[i] = new double[6];
    }
    double mutable_point_for_observations[3];

    mutable_point_for_observations[0] = (double)Point3D.at<float>(0);
    mutable_point_for_observations[1] = (double)Point3D.at<float>(1);
    mutable_point_for_observations[2] = (double)Point3D.at<float>(2);

    std::vector<cv::Mat> rvec;
    for (size_t i = 0; i < size; i++)
    {
        cv::Mat RodVec;
        cv::Rodrigues(matchdata[i].Rotation_world, RodVec);
        rvec.push_back(RodVec);
    }

    for (size_t i = 0; i < size; i++)
    {
        mutable_camera_for_observations[i][0] = (double)rvec[i].at<float>(0);
        mutable_camera_for_observations[i][1] = (double)rvec[i].at<float>(1);
        mutable_camera_for_observations[i][2] = (double)rvec[i].at<float>(2);
        mutable_camera_for_observations[i][3] = (double)matchdata[i].Transform_world.at<float>(0);
        mutable_camera_for_observations[i][4] = (double)matchdata[i].Transform_world.at<float>(1);
        mutable_camera_for_observations[i][5] = (double)matchdata[i].Transform_world.at<float>(2);
    }

    //コスト関数
    for (size_t i = 0; i < size; i++)
    {
        ceres::CostFunction *cost_function_query = ProjectionErrorCostFuctor::Create((double)matchdata[i].image_points.x,
                                                                                     (double)matchdata[i].image_points.y);
        problem.AddResidualBlock(cost_function_query, NULL,
                                 &mutable_camera_for_observations[i][0], &mutable_camera_for_observations[i][1], &mutable_camera_for_observations[i][2],
                                 &mutable_camera_for_observations[i][3], &mutable_camera_for_observations[i][4], &mutable_camera_for_observations[i][5],
                                 &mutable_point_for_observations[0], &mutable_point_for_observations[1], &mutable_point_for_observations[2]);
    }

    // 上限下限の設定
    for (size_t i = 0; i < size; i++)
    {
        problem.SetParameterLowerBound(&mutable_camera_for_observations[i][0], 0, (double)rvec[i].at<float>(0) - 0.05);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[i][1], 0, (double)rvec[i].at<float>(1) - 0.05);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[i][2], 0, (double)rvec[i].at<float>(2) - 0.05);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[i][3], 0, (double)matchdata[i].Transform_world.at<float>(0) - 0.01);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[i][4], 0, (double)matchdata[i].Transform_world.at<float>(1) - 0.01);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[i][5], 0, (double)matchdata[i].Transform_world.at<float>(2) - 0.01);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[i][0], 0, (double)rvec[i].at<float>(0) + 0.05);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[i][1], 0, (double)rvec[i].at<float>(1) + 0.05);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[i][2], 0, (double)rvec[i].at<float>(2) + 0.05);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[i][3], 0, (double)matchdata[i].Transform_world.at<float>(0) + 0.01);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[i][4], 0, (double)matchdata[i].Transform_world.at<float>(1) + 0.01);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[i][5], 0, (double)matchdata[i].Transform_world.at<float>(2) + 0.01);
    }

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = num_CPU_core;

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
    cv::Mat p3_BA(3, 1, CV_32FC1);
    p3_BA.at<float>(0) = (float)mutable_point_for_observations[0];
    p3_BA.at<float>(1) = (float)mutable_point_for_observations[1];
    p3_BA.at<float>(2) = (float)mutable_point_for_observations[2];

    // メモリ解放
    for (size_t i = 0; i < size; i++)
    {
        delete[] mutable_camera_for_observations[i];
    }
    delete[] mutable_camera_for_observations;

    return p3_BA;
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
    cv::Mat Rot_est(3, 3, CV_32FC1), trans_est(3, 1, CV_32FC1);
    R_est.convertTo(Rot_est, CV_32FC1);
    t_est.convertTo(trans_est, CV_32FC1);
    frame_data.camerainfo.Rotation_est = Rot_est.clone();
    frame_data.camerainfo.Transform_est = trans_est.clone();
    frame_data.camerainfo.Rotation_world_est = keyframe_data.camerainfo.Rotation_world * Rot_est;
    frame_data.camerainfo.Transform_world_est = keyframe_data.camerainfo.Rotation_world * trans_est + keyframe_data.camerainfo.Transform_world;
    frame_data.camerainfo.setData_est();

    // 眼球移動量を推定する
    R_move = frame_data.camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world_est;
    t_move = frame_data.camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world_est - frame_data.camerainfo.Transform_world);

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

    for (size_t i = 0; i < pt1.size(); i++)
    {
        std::cout << "p1 : " << pt1[i] << "  p2 : " << pt2[i] << " mask : " << mask.at<bool>(i) << std::endl;
    }
    std::cout << "5点アルゴリズム R_est : " << std::endl
              << Rot_est << std::endl
              << "5点アルゴリズム t_est : " << std::endl
              << trans_est << std::endl
              << "運動学 R : " << std::endl
              << frame_data.camerainfo.Rotation << std::endl
              << "運動学 t :" << std::endl
              << frame_data.camerainfo.Transform << std::endl;

    std::cout << "眼球移動量推定 R_move" << std::endl
              << R_move << std::endl
              << "眼球移動量推定 t_move" << std::endl
              << t_move << std::endl;

    std::cout << "内積 : " << dot_est << std::endl;
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
        cv::Point2f image_center2 = cv::Point2f(frame_data.extractor.image.rows * 3. / 2., frame_data.extractor.image.cols / 2.);
        cv::Scalar color_arrow = cv::Scalar(0, 255, 0);
        cv::Point2f center_t_arm = cv::Point2f(frame_data.camerainfo.Transform.at<float>(0) * 20 + image_center.x,
                                               frame_data.camerainfo.Transform.at<float>(1) * 20 + image_center.y);
        cv::Point2f center_t_arm_z = cv::Point2f(image_center2.x,
                                                 frame_data.camerainfo.Transform.at<float>(2) * 20 + image_center.y);
        cv::arrowedLine(matching_image, image_center, center_t_arm, color_arrow, 2, 8, 0, 0.5);
        cv::arrowedLine(matching_image, image_center2, center_t_arm_z, color_arrow, 2, 8, 0, 0.5);

    }

    // マッチングの様子なしの比較画像を図示
    cv::Mat left_image = keyframe_data.extractor.image;
    cv::Mat right_image = frame_data.extractor.image;
    cv::hconcat(left_image, right_image, nomatching_image);

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