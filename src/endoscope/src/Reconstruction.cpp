#include "../include/endoscope/Reconstruction.hpp"
#include "../include/endoscope/Bundler.hpp"
#include "../include/endoscope/cost_function.hpp"
#include "/home/takeyama/workspace/htl/ros/msg_converter.hpp"
#include "/home/takeyama/workspace/htl/ros/pcl_msg_converter.hpp"
#include "/home/takeyama/workspace/htl/opencv/transform.hpp"
#include "/home/takeyama/workspace/htl/opencv/triangulate.hpp"

Reconstruction::Reconstruction()
    : flag_reconstruction(false), flag_setFirstFrame(true),
      flag_showImage(false), flag_ceres_stdout(false), flag_estimate_move(false), flag_change_showImage(false),
      threshold_knn_ratio(0.7f), threshold_ransac(5.0),
      num_CPU_core(8), num_Scene(KEYPOINT_SCENE),
      matching_method(Matching::BruteForce), extract_type(Extractor::DetectorType::AKAZE),
      use_mode(UseMode::EYE)
{
    switch (use_mode)
    {
    case UseMode::NORMAL_SCENE:
        Z_MAX_CHOOSE = CHOOSE_KF_Z_MAX_N;
        XY_MAX_CHOOSE = CHOOSE_KF_XY_MAX_N;
        XY_MIN_CHOOSE = CHOOSE_KF_XY_MIN_N;
        PHI_MAX_CHOOSE = CHOOSE_KF_PHI_MAX_N;
        XY_MIN_2_CHOOSE = CHOOSE_KF_XY_MIN_2_N;
        Z_MAX_SET = SET_KF_Z_MAX_N;
        XY_MAX_SET = SET_KF_XY_MAX_N;
        PHI_MAX_SET = SET_KF_PHI_MAX_N;
        break;

    case UseMode::EYE:
        Z_MAX_CHOOSE = CHOOSE_KF_Z_MAX_E;
        XY_MAX_CHOOSE = CHOOSE_KF_XY_MAX_E;
        XY_MIN_CHOOSE = CHOOSE_KF_XY_MIN_E;
        PHI_MAX_CHOOSE = CHOOSE_KF_PHI_MAX_E;
        XY_MIN_2_CHOOSE = CHOOSE_KF_XY_MIN_2_E;
        Z_MAX_SET = SET_KF_Z_MAX_E;
        XY_MAX_SET = SET_KF_XY_MAX_E;
        PHI_MAX_SET = SET_KF_PHI_MAX_E;
        break;
    }
}

void Reconstruction::topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                                     const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_normal,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_normal_hold,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_BA,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_BA_hold,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_filtered,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_filtered_hold,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_est,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_est_hold,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_matching_image,
                                     std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_nomatching_image,
                                     std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_keyframe_marker,
                                     std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_matchingframe_marker)
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
    this->publish(pub_pointcloud_normal, pub_pointcloud_normal_hold,
                  pub_pointcloud_BA, pub_pointcloud_BA_hold,
                  pub_pointcloud_filtered, pub_pointcloud_filtered_hold,
                  pub_pointcloud_est, pub_pointcloud_est_hold,
                  pub_matching_image, pub_nomatching_image,
                  pub_keyframe_marker, pub_matchingframe_marker);
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

    if (flag_reconstruction)
    {
        // カメラの位置姿勢を計算
        this->setCameraInfo();

        // 特徴点マッチングと誤対応除去
        this->matching();
        this->outlier_remover();

        // もし眼球移動を検知すれば
        this->estimate_move();

        // 多視点での三角測量
        this->triangulate(); // 三角測量

        // バンドル調整
        // this->bundler(); // バンドル調整]

        // Keyframe_database
        this->updateKeyFrameDatabase();

        // PointCloud
        this->managePointCloud();
    }
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
    flag_change_showImage = false;

    // pointcloud
    point3D_hold.zeros(0, 0, CV_32FC3);
    point3D_BA_hold.zeros(0, 0, CV_32FC3);
    point3D_filtered_hold.zeros(0, 0, CV_32FC3);
    point3D_est_hold.zeros(0, 0, CV_32FC3);
}

void Reconstruction::setFirstFrame()
{
    frame_data.camerainfo.Rotation = Rotation_eye.clone();
    frame_data.camerainfo.Transform = Transform_zeros.clone();

    KeyFrameData temp_keyframedata;
    temp_keyframedata.camerainfo = frame_data.camerainfo;
    temp_keyframedata.extractor = frame_data.extractor;
    keyframe_database.push_back(temp_keyframedata);
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

    // 新しく登録したキーフレームから探索する
    // for (auto itr = keyframe_database.end() - 1; itr != keyframe_database.begin() - 1; --itr)
    for (auto itr = keyframe_database.begin(); itr != keyframe_database.end(); ++itr)
    {
        // 判定条件1: Z方向の変化が少ない
        // カメラ座標での移動量の計算
        cv::Mat t_endo = itr->camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - itr->camerainfo.Transform_world);
        bool moving_z_max = std::abs(t_endo.at<float>(2)) < Z_MAX_CHOOSE;

        // 判定条件2: xy方向の変化or仰角の変化が一定範囲内にある
        // xy方向の移動量bundler
        cv::Point2f t_move_xy(t_endo.at<float>(0), t_endo.at<float>(1));
        bool moving_xy_max = cv::norm(t_move_xy) < XY_MAX_CHOOSE;
        // bool moving_xy_min = cv::norm(t_move_xy) > XY_MIN_CHOOSE;
        // 仰角
        float phi = htl::Transform::RevFromRotMat<float>(itr->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);
        bool moving_phi_max = std::abs(phi) < PHI_MAX_CHOOSE;

        // printf("KF #%d: t_z = %f[%f], xy = %f [%f %f], phi = %f [%f]\n", itr->camerainfo.frame_num, std::abs(t_endo.at<float>(2)), Z_MAX_CHOOSE, cv::norm(t_move_xy), XY_MIN_CHOOSE, XY_MAX_CHOOSE, std::abs(phi), PHI_MAX_CHOOSE);
        if (moving_z_max && moving_xy_max && moving_phi_max)
        {
            // std::cout << "KeyFrame No." << itr->camerainfo.frame_num << " is used (" << itr->scene_counter << ")" << std::endl;
            // std::cout << "moving_z  : " << t_endo.at<float>(2) << " (" << Z_MAX_CHOOSE << ")" << std::endl;
            // std::cout << "moving_xy : " << cv::norm(t_move_xy) << " (" << XY_MIN_CHOOSE << ", " << XY_MAX_CHOOSE << ")" << std::endl;
            // std::cout << "phi : " << phi << " (" << PHI_MAX_CHOOSE << ")" << std::endl;
            itr->scene_counter++;
            itr->camerainfo_dataabase.push_back(frame_data.camerainfo);
            keyframe_itr = itr;
            keyframe_data = *keyframe_itr;
            flag_reconstruction = true;
            flag_change_showImage = true;
            return;
        }
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

    // 新しく登録したキーフレームから探索する(すぐ見つかりやすいので高速になる)
    for (auto itr = keyframe_database.end() - 1; itr != keyframe_database.begin() - 1; --itr)
    {
        // 判定条件1: Z方向の変化が少ない
        // カメラ座標での移動量の計算
        cv::Mat t_endo = itr->camerainfo.Rotation_world.t() * (frame_data.camerainfo.Transform_world - itr->camerainfo.Transform_world);
        if (std::abs(t_endo.at<float>(2)) < Z_MAX_SET)
        {
            // 判定条件2: xy方向の変化or仰角の変化が一定範囲内にある
            // xy方向の移動量
            cv::Point2f t_move_xy(t_endo.at<float>(0), t_endo.at<float>(1));
            bool moving_xy = cv::norm(t_move_xy) < XY_MAX_SET;
            // 仰角
            float phi = htl::Transform::RevFromRotMat<float>(itr->camerainfo.Rotation_world.t() * frame_data.camerainfo.Rotation_world);
            bool moving_phi = std::abs(phi) < PHI_MAX_SET;
            if (moving_xy && moving_phi)
            {
                return;
            }
        }
    }
    // 何一つ当てはまるのがなければKFとする
    frame_data.camerainfo.Rotation = Rotation_eye.clone();
    frame_data.camerainfo.Transform = Transform_zeros.clone();
    KeyFrameData temp_keyframedata;
    temp_keyframedata.camerainfo = frame_data.camerainfo;
    temp_keyframedata.extractor = frame_data.extractor;
    temp_keyframedata.camerainfo_dataabase.push_back(frame_data.camerainfo);
    keyframe_database.push_back(temp_keyframedata);
    printf("KeyFrame was setted!\n");
    return;
}

void Reconstruction::updateKeyFrameDatabase()
{
    // keyframe_databaseの中から抽出したkeyframe_dataを変更する
    // keyframe_databaseからkeyframeを一旦削除し、新たに作成したkeyframe_dataを同じ位置を指定して挿入する
    keyframe_database.erase(keyframe_itr);
    KeyFrameData newKeyFrameData = keyframe_data;
    keyframe_database.insert(keyframe_itr, newKeyFrameData);
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
    frame_data.camerainfo.Rotation_world = htl::Transform::QuaternionToRotMat<float>((float)msg_arm->rotation.x, (float)msg_arm->rotation.y, (float)msg_arm->rotation.z, (float)msg_arm->rotation.w);
    frame_data.camerainfo.Transform_world = (cv::Mat_<float>(3, 1) << msg_arm->translation.x, msg_arm->translation.y, msg_arm->translation.z);
    frame_data.camerainfo.setData();

    // 画像のID情報
    frame_data.camerainfo.frame_num = atoi(msg_image->header.frame_id.c_str());

    // 特徴点検出・特徴量記述
    frame_data.extractor.extractAndcompute(extract_type);
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
    // printf("size : [ %zu, %zu ]\n", inliners_idx.size(), dmatch_num.size()
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
        // その前に過去にこの特徴点がかなり近いフレームから登録されているなら今回は登録しない
        bool flag_insertMap(true);
        typedef std::multimap<int, MatchedData>::iterator iterator;
        std::pair<iterator, iterator> range = keyframe_data.keyponit_map.equal_range(dmatch[num].queryIdx);
        for (iterator itr = range.first; itr != range.second; itr++)
        {
            // 判定条件: xy方向の変化or仰角の変化が一定範囲内にある
            cv::Mat t_endo = itr->second.Rotation_world.t() * (frame_data.camerainfo.Transform_world - itr->second.Transform_world);
            cv::Point2f t_move_xy(t_endo.at<float>(0), t_endo.at<float>(1));
            bool moving_xy_min = cv::norm(t_move_xy) < XY_MIN_2_CHOOSE;
            if (moving_xy_min)
            {
                flag_insertMap = false;
            }
        }
        if (flag_insertMap)
        {
            MatchedData matchData(frame_data.extractor.keypoints[dmatch[num].trainIdx].pt,
                                  frame_data.camerainfo.ProjectionMatrix.clone(),
                                  frame_data.camerainfo.Rotation_world.clone(),
                                  frame_data.camerainfo.Transform_world.clone(),
                                  frame_data.camerainfo.RodriguesVec_world.clone(),
                                  frame_data.camerainfo.frame_num);
            keyframe_data.keyponit_map.insert(std::make_pair(dmatch[num].queryIdx, matchData));
        }
    }
}

void Reconstruction::triangulate()
{
    if (!flag_reconstruction)
        return;

    cv::Mat p3;
    // 今回使ったkeyframeがもつ特徴点毎に辞書を作成しているので、特徴点毎に計算
    for (size_t i = 0; i < keyframe_data.extractor.keypoints.size(); i++)
    {
        // マッチング辞書の中で十分マッチングframeを発見したものを探索
        if (keyframe_data.keyponit_map.count(i) >= num_Scene)
        {
            // printf("keypoint_map.count(%zu) : %zu\n", i, keyframe_data.keyponit_map.count(i));
            // 3次元復元用データコンテナ
            std::vector<cv::Point2f> point2D;
            std::vector<cv::Mat> ProjectMat;

            typedef std::multimap<int, MatchedData>::iterator iterator;
            std::pair<iterator, iterator> range = keyframe_data.keyponit_map.equal_range(i);
            for (iterator itr = range.first; itr != range.second; itr++)
            {
                point2D.push_back(itr->second.image_points);
                ProjectMat.push_back(itr->second.ProjectionMatrix);
            }

            // 三次元復元
            std::vector<bool> eliminated_scene;
            cv::Mat point3D_result = htl::Triangulate::triangulation_RANSAC<float>(point2D, ProjectMat, eliminated_scene, num_Scene);
            // cv::Mat point3D_result = htl::Triangulate::triangulation<float>(point2D, ProjectMat);
            if (!point3D_result.empty())
            {
                // 点の登録
                p3.push_back(point3D_result.reshape(3, 1));

                // RANSACでの三次元復元にて除外されたシーンをkeypoint_mapから削除
                int count = 0;
                for (iterator itr = range.first; itr != range.second;)
                {
                    if (eliminated_scene[count])
                        itr = keyframe_data.keyponit_map.erase(itr);
                    else
                        itr++;
                    count++;
                }

                // バンドル調整用データ
                std::pair<iterator, iterator> range2 = keyframe_data.keyponit_map.equal_range(i);
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
            }

            // 一つの特徴点についてKEYPOINT_SCENE_DELETE個以上のデータが集まれば始まりのほうのものについては削除
            if (keyframe_data.keyponit_map.count(i) >= KEYPOINT_SCENE_DELETE)
            {
                std::pair<iterator, iterator> range3 = keyframe_data.keyponit_map.equal_range(i);
                keyframe_data.keyponit_map.erase(range3.first);
            }
        }
    }
    if (!p3.empty())
    {
        keyframe_data.point_3D.point3D = p3.clone();
    }

    // 点群フィルタ（眼球モードの時だけ起動）
    if (use_mode == UseMode::EYE)
    {
        cv::Mat p3_filter;
        this->pointcloud_eye_filter(keyframe_data.point_3D.point3D, &p3_filter, keyframe_data.camerainfo);
        keyframe_data.point_3D.point3D_filtered = p3_filter.clone();
    }
}

void Reconstruction::managePointCloud()
{
    // pointcloud hold
    cv::Mat p3, p3_BA, p3_filtered, p3_est;
    for (auto itr = keyframe_database.begin(); itr != keyframe_database.end(); itr++)
    {
        p3.push_back(itr->point_3D.point3D);
        p3_BA.push_back(itr->point_3D.point3D_BA);
        p3_filtered.push_back(itr->point_3D.point3D_filtered);
        p3_est.push_back(itr->point_3D.point3D_est);
    }
    point3D_hold = p3.clone();
    point3D_BA_hold = p3_BA.clone();
    point3D_filtered_hold = p3_filtered.clone();
    point3D_est_hold = p3_est.clone();
}

void Reconstruction::bundler()
{
    if (camerainfo_map.empty())
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
    for (auto itr = camerainfo_map.begin(); itr != camerainfo_map.end(); itr++)
    {
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
    for (auto itr = pointData_map.begin(); itr != pointData_map.end(); itr++)
    {
        mutable_point_for_observations[i_point][0] = itr->second.point3D.at<float>(0);
        mutable_point_for_observations[i_point][1] = itr->second.point3D.at<float>(1);
        mutable_point_for_observations[i_point][2] = itr->second.point3D.at<float>(2);
        point2d[i_point][0] = itr->second.point2D.x;
        point2d[i_point][1] = itr->second.point2D.y;

        framenum_point_map.insert(std::make_pair(itr->first, i_point));
        i_point++;
    }

    // コスト関数に追加
    for (auto frame_itr = framenum_cam_map.begin(); frame_itr != framenum_cam_map.end(); frame_itr++)
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

    // パラメータの最適化における上限下限設定
    for (auto frame_itr = framenum_cam_map.begin(); frame_itr != framenum_cam_map.end(); frame_itr++)
    {
        int access_num_cam = framenum_cam_map.at(frame_itr->first);
        // 下限
        problem.SetParameterLowerBound(&mutable_camera_for_observations[access_num_cam][0], 0, camerainfo_map.at(frame_itr->first).RodriguesVec_world.at<float>(0) - 0.05);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[access_num_cam][1], 0, camerainfo_map.at(frame_itr->first).RodriguesVec_world.at<float>(1) - 0.05);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[access_num_cam][2], 0, camerainfo_map.at(frame_itr->first).RodriguesVec_world.at<float>(2) - 0.05);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[access_num_cam][3], 0, camerainfo_map.at(frame_itr->first).Transform_world.at<float>(0) - 0.01);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[access_num_cam][4], 0, camerainfo_map.at(frame_itr->first).Transform_world.at<float>(1) - 0.01);
        problem.SetParameterLowerBound(&mutable_camera_for_observations[access_num_cam][5], 0, camerainfo_map.at(frame_itr->first).Transform_world.at<float>(2) - 0.01);
        // 上限
        problem.SetParameterUpperBound(&mutable_camera_for_observations[access_num_cam][0], 0, camerainfo_map.at(frame_itr->first).RodriguesVec_world.at<float>(0) + 0.05);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[access_num_cam][1], 0, camerainfo_map.at(frame_itr->first).RodriguesVec_world.at<float>(1) + 0.05);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[access_num_cam][2], 0, camerainfo_map.at(frame_itr->first).RodriguesVec_world.at<float>(2) + 0.05);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[access_num_cam][3], 0, camerainfo_map.at(frame_itr->first).Transform_world.at<float>(0) + 0.01);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[access_num_cam][4], 0, camerainfo_map.at(frame_itr->first).Transform_world.at<float>(1) + 0.01);
        problem.SetParameterUpperBound(&mutable_camera_for_observations[access_num_cam][5], 0, camerainfo_map.at(frame_itr->first).Transform_world.at<float>(2) + 0.01);
    }

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = flag_ceres_stdout;
    options.num_threads = num_CPU_core;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // publish用データ
    cv::Mat p3;
    for (size_t i = 0; i < pointData_map.size(); i++)
    {
        cv::Mat p3_BA(3, 1, CV_32FC1);
        p3_BA.at<float>(0) = mutable_point_for_observations[i][0];
        p3_BA.at<float>(1) = mutable_point_for_observations[i][1];
        p3_BA.at<float>(2) = mutable_point_for_observations[i][2];
        p3.push_back(p3_BA.reshape(3, 1));
    }
    keyframe_data.point_3D.point3D_BA = p3.clone();

    // メモリ解放
    for (size_t i = 0; i < camerainfo_map.size(); i++)
    {
        delete[] mutable_camera_for_observations[i];
    }
    for (size_t i = 0; i < pointData_map.size(); i++)
    {
        delete[] mutable_point_for_observations[i];
        delete[] point2d[i];
    }
}

void Reconstruction::pointcloud_eye_filter(const cv::Mat &InputPoint3D, cv::Mat *OutputPoint3D, const CameraInfo &camera_state)
{
    int point_num = InputPoint3D.rows;
    if (point_num == 0)
        return;

    cv::Mat temp_Point3D, temp_Point3D_cam;
    // 内視鏡からの距離が24mm以内かつカメラ視線方向にあるかどうか一点ごとに検査
    for (int i = 0; i < point_num; i++)
    {
        cv::Mat point_world(3, 1, CV_32FC1);
        point_world.at<float>(0) = InputPoint3D.at<cv::Point3f>(i).x;
        point_world.at<float>(1) = InputPoint3D.at<cv::Point3f>(i).y;
        point_world.at<float>(2) = InputPoint3D.at<cv::Point3f>(i).z;
        float distance = std::sqrt((point_world.at<float>(0) - camera_state.Transform_world.at<float>(0)) * (point_world.at<float>(0) - camera_state.Transform_world.at<float>(0)) +
                                   (point_world.at<float>(1) - camera_state.Transform_world.at<float>(1)) * (point_world.at<float>(1) - camera_state.Transform_world.at<float>(1)) +
                                   (point_world.at<float>(2) - camera_state.Transform_world.at<float>(2)) * (point_world.at<float>(2) - camera_state.Transform_world.at<float>(2)));
        cv::Mat point_cam = camera_state.Rotation_world.t() * (point_world - camera_state.Transform_world);
        // std::cout << "distance : " << distance << std::endl;
        // std::cout << "point_cam : " << point_cam << std::endl;
        if (distance < THRESH_DISTANCE_EYE && point_cam.at<float>(2) > 0.001)
        {
            temp_Point3D.push_back(InputPoint3D.at<cv::Point3f>(i));
        }
        cv::Point3f pt;
        pt.x = point_cam.at<float>(0);
        pt.y = point_cam.at<float>(1);
        pt.z = point_cam.at<float>(2);
        temp_Point3D_cam.push_back(pt);
    }

    // カメラ座標系から見た点群のz方向の分散が大きかったら除外
    // 平均
    cv::Point3f point_average;
    for (int i = 0; i < temp_Point3D.rows; i++)
    {
        point_average.z += temp_Point3D.at<cv::Vec3f>(i, 0)[2];
    }
    point_average.z /= temp_Point3D.rows;
    // 分散
    cv::Point3f point_variance;
    for (int i = 0; i < temp_Point3D.rows; i++)
    {
        point_variance.z += (temp_Point3D.at<cv::Vec3f>(i, 0)[2] - point_average.z) * (temp_Point3D.at<cv::Vec3f>(i, 0)[2] - point_average.z);
    }
    point_average.z /= temp_Point3D.rows;
    // printf("var:[%f]\n", point_variance.z);

    //分散がでかすぎたらアウト
    if (point_variance.z < THRESH_VARIANCE_POINT)
    {
        *OutputPoint3D = temp_Point3D;
    }
}

void Reconstruction::estimate_move()
{
    // 眼球の移動を検知したらフラグを管理する
    if (!flag_estimate_move)
        return;

    // 5点アルゴリズムやる以上、5点以上の点が必要
    if (matched_point1.size() < 5)
        return;

    std::cout << "Estimating Eye Moving : point.size = " << matched_point1.size() << std::endl;

    // 画像座標から正規化カメラ座標系に変換
    // focalについては無視（よくわからんけどこれで動くのでヨシ！）
    std::vector<cv::Point2f> pt1, pt2;
    for (size_t i = 0; i < matched_point1.size(); i++)
    {
        cv::Point2f point1, point2;
        point1.x = matched_point1[i].x - PP_X;
        point1.y = matched_point1[i].y - PP_Y;
        point2.x = matched_point2[i].x - PP_X;
        point2.y = matched_point2[i].y - PP_Y;
        pt1.push_back(point1);
        pt2.push_back(point2);
    }

    // 5点アルゴリズム
    // １つめの画像を正規化座標としたときに2枚目の画像への回転・並進変換行列
    cv::Mat R_est_output, t_est_output, mask;
    cv::Point2d principle_point(PP_X, PP_Y);
    cv::Mat EssentialMat = cv::findEssentialMat(pt1, pt2, 1, cv::Point2f(0, 0), cv::RANSAC, 0.99, 1, mask);
    if (!(EssentialMat.rows == 3 && EssentialMat.cols == 3))
    {
        printf("Error: EssentialMat.rows = %d, EssentialMat.cols = %d\n", EssentialMat.rows, EssentialMat.cols);
        return;
    }
    cv::recoverPose(EssentialMat, pt1, pt2, R_est_output, t_est_output, 1, cv::Point2f(0, 0), mask);

    if (R_est_output.at<double>(0, 0) < 0.8 || R_est_output.at<double>(1, 1) < 0.8 || R_est_output.at<double>(2, 2) < 0.8)
    {
        std::cout << "Error: Move Estimated, but output is incorrect estimation." << std::endl;
        // std::cout << "Incorrect R_estimate : " << R_est_output << std::endl;
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
    if (dot_est < THRESH_DOT)
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
    std::cout << "5点アルゴリズム R_est : " << std::endl
              << Rot_est << std::endl
              << "5点アルゴリズム t_est : " << std::endl
              << trans_est << std::endl
              << "運動学 R : " << std::endl
              << frame_data.camerainfo.Rotation << std::endl
              << "運動学 t :" << std::endl
              << frame_data.camerainfo.Transform << std::endl
              << "眼球移動量推定 R_eye_move" << std::endl
              << R_eye_move << std::endl
              << "眼球移動量推定 t_eye_move" << std::endl
              << t_eye_move << std::endl;

    std::cout << "内積 : " << dot_est << std::endl;
}

void Reconstruction::showImage()
{
    if (!flag_change_showImage)
    {
        return;
    }

    if (!inliners_matches.empty())
    {
        // マッチングの様子を図示
        cv::Scalar match_line_color = cv::Scalar(255, 0, 0);
        cv::Scalar match_point_color = cv::Scalar(255, 255, 0);
        std::vector<char> matchesMask;
        cv::drawMatches(keyframe_data.extractor.image, keyframe_data.extractor.keypoints,
                        frame_data.extractor.image, frame_data.extractor.keypoints,
                        inliners_matches, matching_image, match_line_color, match_point_color, matchesMask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // フレーム番号も記載
        char numStr_keyframe_framenum[10];
        char numStr_frame_framenum[10];
        sprintf(numStr_keyframe_framenum, "#%d", keyframe_data.camerainfo.frame_num);
        sprintf(numStr_frame_framenum, "#%d", frame_data.camerainfo.frame_num);
        cv::putText(matching_image, numStr_keyframe_framenum, cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 200), 2, 16);
        cv::putText(matching_image, numStr_frame_framenum, cv::Point(500, 300), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 200), 2, 16);

        // 表示
        if (!matching_image.empty() && flag_showImage)
        {
            cv::imshow("matching_image", matching_image);
            cv::waitKey(1);
        }
    }

    if (!keyframe_data.extractor.image.empty() && !frame_data.extractor.image.empty())
    {
        // マッチングの様子なしの比較画像を図示
        cv::Mat temp_nomatching_image;
        cv::Mat left_image = keyframe_data.extractor.image.clone();
        cv::Mat right_image = frame_data.extractor.image.clone();
        cv::hconcat(left_image, right_image, temp_nomatching_image);

        // カメラのuv方向への移動量を矢印で追加で図示
        cv::Point2f image_center = cv::Point2f(frame_data.extractor.image.rows / 2., frame_data.extractor.image.cols / 2.);
        cv::Scalar color_arrow = cv::Scalar(0, 0, 255);
        // ロールピッチの回転による移動量
        // cv::Vec3f EulerAngles = htl::Transform::RotMatToEulerAngles<float>(frame_data.camerainfo.Rotation);
        // float x_pitch = LENGTH_ENDOSCOPE * tan(EulerAngles[1]);
        // float y_roll = -1 * LENGTH_ENDOSCOPE * tan(EulerAngles[0]);
        // std::cout << "Transform : " << frame_data.camerainfo.Transform << std::endl;
        // std::cout << "rall, pitch : [" << x_pitch << ", " << y_roll << "]" << std::endl;
        cv::Point2f center_t_arm = cv::Point2f((frame_data.camerainfo.Transform.at<float>(0)) * 100000 + image_center.x,
                                               (frame_data.camerainfo.Transform.at<float>(1)) * 100000 + image_center.y);
        cv::arrowedLine(temp_nomatching_image, image_center, center_t_arm, color_arrow, 3, 8, 0, 0.5);

        // フレーム番号も記載
        char numStr_keyframe_framenum[10];
        char numStr_frame_framenum[10];
        sprintf(numStr_keyframe_framenum, "#%d", keyframe_data.camerainfo.frame_num);
        sprintf(numStr_frame_framenum, "#%d", frame_data.camerainfo.frame_num);
        cv::putText(temp_nomatching_image, numStr_keyframe_framenum, cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 200), 2, 16);
        cv::putText(temp_nomatching_image, numStr_frame_framenum, cv::Point(500, 300), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 200), 2, 16);

        // 眼球移動量推定を行っていればその結果も矢印で表記
        if (!t_eye_move.empty())
        {
            cv::Point2f image_center2 = cv::Point2f(frame_data.extractor.image.rows * 3. / 2., frame_data.extractor.image.cols / 2.);
            cv::Point2f center_t_arm_est = cv::Point2f(trans_est.at<float>(0) * 2500 + image_center2.x,
                                                       trans_est.at<float>(1) * 2500 + image_center2.y);
            cv::arrowedLine(temp_nomatching_image, image_center2, center_t_arm_est, color_arrow, 2, 8, 0, 0.5);
        }
        nomatching_image = temp_nomatching_image.clone();

        // 表示
        if (!temp_nomatching_image.empty() && flag_showImage)
        {
            cv::imshow("nomatching_image", nomatching_image);
            cv::waitKey(1);
        }
    }
}

void Reconstruction::publish(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_normal,
                             std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_normal_hold,
                             std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_BA,
                             std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_BA_hold,
                             std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_filtered,
                             std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_filtered_hold,
                             std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_est,
                             std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_est_hold,
                             std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_matching_image,
                             std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_nomatching_image,
                             std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_keyframe_marker,
                             std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_matchingframe_marker)
{
    if (flag_setFirstFrame)
    {
        flag_setFirstFrame = false;
        return;
    }

    // Pointcloud
    cv::Mat pointCloud_normal;
    keyframe_data.point_3D.point3D.convertTo(pointCloud_normal, CV_32FC3);
    auto msg_cloud_normal_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    htl::PCL_Converter::cvMat_to_msgPointCloud(pointCloud_normal, *msg_cloud_normal_pub, htl::PCL_Converter::Color::BLUE);
    pub_pointcloud_normal->publish(*msg_cloud_normal_pub);

    cv::Mat pointCloud_normal_hold;
    point3D_hold.convertTo(pointCloud_normal_hold, CV_32FC3);
    auto msg_cloud_normal_hold_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    htl::PCL_Converter::cvMat_to_msgPointCloud(pointCloud_normal_hold, *msg_cloud_normal_hold_pub, htl::PCL_Converter::Color::YELLOW);
    pub_pointcloud_normal_hold->publish(*msg_cloud_normal_hold_pub);

    cv::Mat pointCloud_BA;
    keyframe_data.point_3D.point3D_BA.convertTo(pointCloud_BA, CV_32FC3);
    auto msg_cloud_BA_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    htl::PCL_Converter::cvMat_to_msgPointCloud(pointCloud_BA, *msg_cloud_BA_pub, htl::PCL_Converter::Color::GREEN);
    pub_pointcloud_BA->publish(*msg_cloud_BA_pub);

    cv::Mat pointCloud_BA_hold;
    point3D_BA_hold.convertTo(pointCloud_BA_hold, CV_32FC3);
    auto msg_cloud_BA_hold_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    htl::PCL_Converter::cvMat_to_msgPointCloud(pointCloud_BA_hold, *msg_cloud_BA_hold_pub, htl::PCL_Converter::Color::RED);
    pub_pointcloud_BA_hold->publish(*msg_cloud_BA_hold_pub);

    cv::Mat pointCloud_filtered;
    keyframe_data.point_3D.point3D_filtered.convertTo(pointCloud_filtered, CV_32FC3);
    auto msg_cloud_filtered_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    htl::PCL_Converter::cvMat_to_msgPointCloud(pointCloud_filtered, *msg_cloud_filtered_pub, htl::PCL_Converter::Color::AQUA);
    pub_pointcloud_filtered->publish(*msg_cloud_filtered_pub);

    cv::Mat pointCloud_filtered_hold;
    point3D_filtered_hold.convertTo(pointCloud_filtered_hold, CV_32FC3);
    auto msg_cloud_filtered_hold_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    htl::PCL_Converter::cvMat_to_msgPointCloud(pointCloud_filtered_hold, *msg_cloud_filtered_hold_pub, htl::PCL_Converter::Color::RED);
    pub_pointcloud_filtered_hold->publish(*msg_cloud_filtered_hold_pub);

    cv::Mat pointCloud_est;
    keyframe_data.point_3D.point3D_est.convertTo(pointCloud_est, CV_32FC3);
    auto msg_cloud_est_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    htl::PCL_Converter::cvMat_to_msgPointCloud(pointCloud_est, *msg_cloud_est_pub, htl::PCL_Converter::Color::TEAL);
    pub_pointcloud_est->publish(*msg_cloud_est_pub);

    cv::Mat pointCloud_est_hold;
    point3D_est_hold.convertTo(pointCloud_est_hold, CV_32FC3);
    auto msg_cloud_est_hold_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    htl::PCL_Converter::cvMat_to_msgPointCloud(pointCloud_est_hold, *msg_cloud_est_hold_pub, htl::PCL_Converter::Color::OLIVE);
    pub_pointcloud_est_hold->publish(*msg_cloud_est_hold_pub);

    // Image
    if (!matching_image.empty())
    {
        auto msg_matching_image = std::make_shared<sensor_msgs::msg::Image>();
        this->convert_frame_to_message(matching_image, frame_data.camerainfo.frame_num, *msg_matching_image);
        pub_matching_image->publish(std::move(*msg_matching_image));
    }

    if (!nomatching_image.empty())
    {
        auto msg_nomatching_image = std::make_shared<sensor_msgs::msg::Image>();
        this->convert_frame_to_message(nomatching_image, frame_data.camerainfo.frame_num, *msg_nomatching_image);
        pub_nomatching_image->publish(std::move(*msg_nomatching_image));
    }

    // keyframe
    if (keyframe_database.size() > 0)
    {
        auto msg_keyframe_marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
        for (auto itr = keyframe_database.begin(); itr != keyframe_database.end(); itr++)
        {
            if (itr->camerainfo.frame_num != keyframe_data.camerainfo.frame_num)
            {
                visualization_msgs::msg::Marker msg_marker;
                msg_marker.header.frame_id = "world";
                msg_marker.ns = "keyframe #" + std::to_string(itr->camerainfo.frame_num);
                msg_marker.type = visualization_msgs::msg::Marker::CUBE;
                msg_marker.action = visualization_msgs::msg::Marker::ADD;
                // 大きさ
                msg_marker.scale.x = 0.001;
                msg_marker.scale.y = 0.001;
                msg_marker.scale.z = 0.0001;
                // 色
                msg_marker.color.r = 0.0;
                msg_marker.color.g = 1.0;
                msg_marker.color.b = 1.0;
                msg_marker.color.a = 0.4;
                // 位置・姿勢
                msg_marker.pose.position.x = (double)itr->camerainfo.Transform_world.at<float>(0);
                msg_marker.pose.position.y = (double)itr->camerainfo.Transform_world.at<float>(1);
                msg_marker.pose.position.z = (double)itr->camerainfo.Transform_world.at<float>(2);
                cv::Vec4f temp_q;
                htl::Transform::RotMatToQuaternion<float>(&temp_q[0], &temp_q[1], &temp_q[2], &temp_q[3],
                                                          itr->camerainfo.Rotation_world.at<float>(0, 0), itr->camerainfo.Rotation_world.at<float>(0, 1), itr->camerainfo.Rotation_world.at<float>(0, 2),
                                                          itr->camerainfo.Rotation_world.at<float>(1, 0), itr->camerainfo.Rotation_world.at<float>(1, 1), itr->camerainfo.Rotation_world.at<float>(1, 2),
                                                          itr->camerainfo.Rotation_world.at<float>(2, 0), itr->camerainfo.Rotation_world.at<float>(2, 1), itr->camerainfo.Rotation_world.at<float>(2, 2));
                msg_marker.pose.orientation.x = temp_q[0];
                msg_marker.pose.orientation.y = temp_q[1];
                msg_marker.pose.orientation.z = temp_q[2];
                msg_marker.pose.orientation.w = temp_q[3];
                msg_keyframe_marker_array->markers.push_back(msg_marker);
            }
        }
        pub_keyframe_marker->publish(*msg_keyframe_marker_array);
    }

    // matchingframe
    if (flag_reconstruction)
    {
        auto msg_matching_marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
        /* keyframe側 */
        visualization_msgs::msg::Marker msg_marker;
        msg_marker.header.frame_id = "world";
        msg_marker.ns = "keyframe";
        msg_marker.type = visualization_msgs::msg::Marker::CUBE;
        msg_marker.action = visualization_msgs::msg::Marker::ADD;
        // 大きさ
        msg_marker.scale.x = 0.001;
        msg_marker.scale.y = 0.001;
        msg_marker.scale.z = 0.0001;
        // 色
        msg_marker.color.r = 1.0;
        msg_marker.color.g = 0.0;
        msg_marker.color.b = 0.0;
        msg_marker.color.a = 0.8;
        // 位置・姿勢
        msg_marker.pose.position.x = (double)keyframe_data.camerainfo.Transform_world.at<float>(0);
        msg_marker.pose.position.y = (double)keyframe_data.camerainfo.Transform_world.at<float>(1);
        msg_marker.pose.position.z = (double)keyframe_data.camerainfo.Transform_world.at<float>(2);
        cv::Vec4f temp_q;
        htl::Transform::RotMatToQuaternion<float>(&temp_q[0], &temp_q[1], &temp_q[2], &temp_q[3],
                                                  keyframe_data.camerainfo.Rotation_world.at<float>(0, 0), keyframe_data.camerainfo.Rotation_world.at<float>(0, 1), keyframe_data.camerainfo.Rotation_world.at<float>(0, 2),
                                                  keyframe_data.camerainfo.Rotation_world.at<float>(1, 0), keyframe_data.camerainfo.Rotation_world.at<float>(1, 1), keyframe_data.camerainfo.Rotation_world.at<float>(1, 2),
                                                  keyframe_data.camerainfo.Rotation_world.at<float>(2, 0), keyframe_data.camerainfo.Rotation_world.at<float>(2, 1), keyframe_data.camerainfo.Rotation_world.at<float>(2, 2));
        msg_marker.pose.orientation.x = temp_q[0];
        msg_marker.pose.orientation.y = temp_q[1];
        msg_marker.pose.orientation.z = temp_q[2];
        msg_marker.pose.orientation.w = temp_q[3];
        msg_matching_marker_array->markers.push_back(msg_marker);

        /* frame側 */
        // 位置・姿勢(位置・姿勢と名前以外同じなので、それだけ変更)
        msg_marker.ns = "now_frame";
        msg_marker.pose.position.x = (double)frame_data.camerainfo.Transform_world.at<float>(0);
        msg_marker.pose.position.y = (double)frame_data.camerainfo.Transform_world.at<float>(1);
        msg_marker.pose.position.z = (double)frame_data.camerainfo.Transform_world.at<float>(2);
        cv::Vec4f temp_q2;
        htl::Transform::RotMatToQuaternion<float>(&temp_q2[0], &temp_q2[1], &temp_q2[2], &temp_q2[3],
                                                  frame_data.camerainfo.Rotation_world.at<float>(0, 0), frame_data.camerainfo.Rotation_world.at<float>(0, 1), frame_data.camerainfo.Rotation_world.at<float>(0, 2),
                                                  frame_data.camerainfo.Rotation_world.at<float>(1, 0), frame_data.camerainfo.Rotation_world.at<float>(1, 1), frame_data.camerainfo.Rotation_world.at<float>(1, 2),
                                                  frame_data.camerainfo.Rotation_world.at<float>(2, 0), frame_data.camerainfo.Rotation_world.at<float>(2, 1), frame_data.camerainfo.Rotation_world.at<float>(2, 2));
        msg_marker.pose.orientation.x = temp_q2[0];
        msg_marker.pose.orientation.y = temp_q2[1];
        msg_marker.pose.orientation.z = temp_q2[2];
        msg_marker.pose.orientation.w = temp_q2[3];
        msg_matching_marker_array->markers.push_back(msg_marker);

        // Publish
        pub_matchingframe_marker->publish(*msg_matching_marker_array);
    }
}

std::string Reconstruction::mat_type2encoding(int mat_type)
{
    switch (mat_type)
    {
    case CV_8UC1:
        return "mono8";
    case CV_8UC3:
        return "bgr8";
    case CV_16SC1:
        return "mono16";
    case CV_8UC4:
        return "rgba8";
    default:
        throw std::runtime_error("Unsupported encoding type");
    }
}

void Reconstruction::convert_frame_to_message(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg)
{
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    msg.is_bigendian = false;
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = std::to_string(frame_id);
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

void Reconstruction::setUseMode(size_t num)
{
    this->use_mode = num;

    switch (use_mode)
    {
    case UseMode::NORMAL_SCENE:
        Z_MAX_CHOOSE = CHOOSE_KF_Z_MAX_N;
        XY_MAX_CHOOSE = CHOOSE_KF_XY_MAX_N;
        XY_MIN_CHOOSE = CHOOSE_KF_XY_MIN_N;
        PHI_MAX_CHOOSE = CHOOSE_KF_PHI_MAX_N;
        Z_MAX_SET = SET_KF_Z_MAX_N;
        XY_MAX_SET = SET_KF_XY_MAX_N;
        PHI_MAX_SET = SET_KF_PHI_MAX_N;
        break;

    case UseMode::EYE:
        Z_MAX_CHOOSE = CHOOSE_KF_Z_MAX_E;
        XY_MAX_CHOOSE = CHOOSE_KF_XY_MAX_E;
        XY_MIN_CHOOSE = CHOOSE_KF_XY_MIN_E;
        PHI_MAX_CHOOSE = CHOOSE_KF_PHI_MAX_E;
        Z_MAX_SET = SET_KF_Z_MAX_E;
        XY_MAX_SET = SET_KF_XY_MAX_E;
        PHI_MAX_SET = SET_KF_PHI_MAX_E;
        break;
    }
}

void Reconstruction::setMatchingMethod(size_t num)
{
    this->matching_method = num;
}

void Reconstruction::setExtractor(size_t num)
{
    this->extract_type = num;
}

void Reconstruction::setFlagCeresstdout(bool flag)
{
    this->flag_ceres_stdout = flag;
}