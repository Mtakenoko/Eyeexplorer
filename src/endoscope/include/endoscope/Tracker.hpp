#ifndef TRACKER_HPP__
#define TRACKER_HPP__

#include <iostream>
#include <unordered_map>
#include <string>

#include <opencv2/opencv.hpp>

class LastFrame
{
public:
    LastFrame(int i1, float x_1, float y_1, int i2, float x_2, float y_2, unsigned int track_num, std::string frame_num)
        : idx1(i1), x1(x_1), y1(y_1), idx2(i2), x2(x_2), y2(y_2), track_no(track_num), frame_id(frame_num) {}

public:
    int idx1;
    float x1, y1;
    int idx2;
    float x2, y2;
    unsigned int track_no;
    std::string frame_id;
};

class Tracker
{
public:
    Tracker()
        : track_no(0), threshold_ratio(0.7f), threshold_ratio_other(0.7f), threshold_ransac(5.), threshold_track_no(5000), flag_showimage(false), flag_setfirstframe(true){};

    Tracker(const int _detector, const int _matcher)
        : track_no(0), threshold_ratio(0.7f), threshold_ratio_other(0.7f), threshold_ransac(5.), threshold_track_no(5000), flag_showimage(false), flag_setfirstframe(true)
    {
        setDetector(_detector);
        setMatcher(_matcher);
    };
    ~Tracker() { printf("Tracker distructor was called\n"); };

public:
    void setDetector(int feature);
    void setMatcher(int matcherType);
    void setFirstFrame(cv::Mat frame);
    void setThreshold_MatchRatio(float input_threshold_ratio) { threshold_ratio = input_threshold_ratio; };
    void setThreshold_MatchRatio_other(float input_threshold_ratio_other) { threshold_ratio_other = input_threshold_ratio_other; };
    void setThreshold_RANSAC(float input_threshold_ransac) { threshold_ransac = input_threshold_ransac; };
    void setThreshold_TrackNo(unsigned int input_threshold_trackno) { threshold_track_no = input_threshold_trackno; };
    void setFlagShowImage(bool showimage) { flag_showimage = showimage; };
    void setFrame_Id(std::string frame_id_) { frame_id = frame_id_; };

    cv::Mat getMatchedImage() { return matched_image; };
    std::vector<std::map<int, LastFrame>> getLastFrame() { return last_frame; };
    std::vector<std::map<unsigned int, LastFrame>> getFindTrack() { return find_track; };

    void process(const cv::Mat frame);
    void showMatchedImage();
    void calcFindTrack();

    enum DetectorType : int
    {
        AKAZE = 0,
        ORB = 1,
        BRISK = 2,
        SIFT = 3,
        SURF = 4
    };

    enum MatcerType
    {
        FLANNBASED = 0,
        BRUTEFORCED = 1
    };

private:
    void resetFirstFrame(cv::Mat reset_frame);

    cv::Ptr<cv::Feature2D> detector;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Mat first_frame, first_descriptor;
    std::vector<cv::KeyPoint> first_keypoint;
    std::vector<cv::Point2f> object_bb;
    cv::Mat matched_image;
    std::vector<std::map<int, LastFrame>> last_frame;
    std::vector<std::map<unsigned int, LastFrame>> find_track;
    unsigned int track_no;
    std::string frame_id;
    float threshold_ratio;
    float threshold_ratio_other;
    float threshold_ransac;
    unsigned int threshold_track_no;
    bool flag_showimage;
    bool flag_setfirstframe;
};
#endif

void Tracker::setDetector(int featureType)
{
    if (featureType == Tracker::AKAZE)
    {
        detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0000001f);
        /***覚書****
        ・DESCRIPTOR_MLDBのMLDBは「modified-local differnce 
        ・UPRIGHTは回転不変でない記述子なので、使わないほうが良さそう。
        *********/
    }
    else if (featureType == Tracker::ORB)
    {
        detector = cv::ORB::create(1000, 1.2f, 30, 31, 0, 2, cv::ORB::FAST_SCORE, 31, 5);
    }
    else if (featureType == Tracker::BRISK)
    {
        detector = cv::BRISK::create(120, 3, 0.6f);
    }
    else
    {
        printf("Choosing Incorrect Option of Feature point detector.\n");
        return;
    }
}
void Tracker::setMatcher(int matcherType)
{
    if (matcherType == Tracker::FLANNBASED)
    {
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }
    else if (matcherType == Tracker::BRUTEFORCED)
    {
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    }
    else
    {
        printf("Choosing Incorrect Option of Feature point matcher.\n");
        return;
    }
}

void Tracker::setFirstFrame(cv::Mat frame)
{
    if (frame.empty())
    {
        printf("First frame is empty\n");
        return;
    }
    flag_setfirstframe = false;
    first_frame = frame.clone();
    detector->detectAndCompute(first_frame, cv::noArray(), first_keypoint, first_descriptor);
    if (first_descriptor.type() != CV_32F)
        first_descriptor.convertTo(first_descriptor, CV_32F);
}

void Tracker::resetFirstFrame(cv::Mat reset_frame)
{
    flag_setfirstframe = true;
    first_frame.release();
    first_keypoint.clear();
    first_descriptor.release();
    setFirstFrame(reset_frame);
    printf("FirstFrame was setted to frame!\n");
}

std::vector<cv::Point2f> keypoint2Point(std::vector<cv::KeyPoint> kp)
{
    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < kp.size(); i++)
    {
        points.push_back(kp[i].pt);
    }
    return points;
}

void Tracker::showMatchedImage()
{
    if (!matched_image.empty() && flag_showimage)
    {
        cv::imshow("matched_image", matched_image);
        cv::waitKey(5);
    }
}

void Tracker::process(const cv::Mat frame)
{
    if (flag_setfirstframe)
    {
        setFirstFrame(frame);
        printf("FirstFrame was setted!\n");
        return;
    }
    if (first_frame.empty() || frame.empty())
    {
        printf("frame is empty!\n");
        rclcpp::shutdown();
    }

    //　特徴点の検出、特徴量記述
    std::vector<cv::KeyPoint> keypoint;
    cv::Mat descriptor;
    detector->detectAndCompute(frame, cv::noArray(), keypoint, descriptor);
    if (descriptor.type() != CV_32F)
        descriptor.convertTo(descriptor, CV_32F);

    // マッチング
    std::vector<std::vector<cv::DMatch>> knn_matches;
    std::vector<cv::KeyPoint> matched1_keypoints, matched2_keypoints;
    std::vector<cv::DMatch> dmatch;
    std::vector<int> matched1_keypoints_idx, matched2_keypoints_idx;
    matched1_keypoints_idx.clear();
    matched2_keypoints_idx.clear();
    matcher->knnMatch(first_descriptor, descriptor, knn_matches, 3);

    // 誤対応除去①
    // ハミング距離がある一定範囲よりも近いものだけ選択
    if (knn_matches.size() == 0)
    {
        resetFirstFrame(frame);
        return;
    }
    if (!last_frame.empty())
    {
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            // 一つ前のフレーム内でのidx2と値が等しい
            int use_knnnum;
            int idx1_0 = knn_matches[i][0].queryIdx;
            auto itr_0 = last_frame.back().find(idx1_0);
            if (itr_0 != last_frame.back().end())
            {
                use_knnnum = 0;
            }
            else
            {
                int idx1_1 = knn_matches[i][1].queryIdx;
                auto itr_1 = last_frame.back().find(idx1_1);
                if (itr_1 != last_frame.back().end())
                {
                    use_knnnum = 1;
                }
                else
                {
                    int idx1_2 = knn_matches[i][2].queryIdx;
                    auto itr_2 = last_frame.back().find(idx1_2);
                    if (itr_2 != last_frame.back().end())
                    {
                        use_knnnum = 2;
                    }
                    else
                    {
                        use_knnnum = -1;
                    }
                }
            }
            if ((use_knnnum == 0 || use_knnnum == -1) && knn_matches[i][0].distance < threshold_ratio * knn_matches[i][1].distance)
            {
                // printf("use point [%zu, %d]\n", i, use_knnnum);
                // i番目のマッチングについてのクエリディスクリプタが存在するキーポイント（first_keypointの中で）
                matched1_keypoints.push_back(first_keypoint[knn_matches[i][0].queryIdx]);
                matched2_keypoints.push_back(keypoint[knn_matches[i][0].trainIdx]);
                // i番目のマッチングについてのクエリディスクリプタのインデックス
                matched1_keypoints_idx.push_back(knn_matches[i][0].queryIdx);
                matched2_keypoints_idx.push_back(knn_matches[i][0].trainIdx);
                dmatch.push_back(knn_matches[i][0]);
            }
            else if ((use_knnnum == 1 || use_knnnum == 2) && knn_matches[i][0].distance > threshold_ratio_other * knn_matches[i][use_knnnum].distance)
            {
                // printf("うおおおおuse unliked point [%zu, %d]\n", i, use_knnnum);
                // i番目のマッチングについてのクエリディスクリプタが存在するキーポイント（first_keypointの中で）
                matched1_keypoints.push_back(first_keypoint[knn_matches[i][use_knnnum].queryIdx]);
                matched2_keypoints.push_back(keypoint[knn_matches[i][use_knnnum].trainIdx]);
                // i番目のマッチングについてのクエリディスクリプタのインデックス
                matched1_keypoints_idx.push_back(knn_matches[i][use_knnnum].queryIdx);
                matched2_keypoints_idx.push_back(knn_matches[i][use_knnnum].trainIdx);
                dmatch.push_back(knn_matches[i][use_knnnum]);
            }
        }
    }
    else
    {
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                // i番目のマッチングについてのクエリディスクリプタが存在するキーポイント（first_keypointの中で）
                matched1_keypoints.push_back(first_keypoint[knn_matches[i][j].queryIdx]);
                matched2_keypoints.push_back(keypoint[knn_matches[i][j].trainIdx]);
                // i番目のマッチングについてのクエリディスクリプタのインデックス
                matched1_keypoints_idx.push_back(knn_matches[i][j].queryIdx);
                matched2_keypoints_idx.push_back(knn_matches[i][j].trainIdx);
                dmatch.push_back(knn_matches[i][j]);
            }
        }
    }

    // 誤対応除去②
    // ホモグラフィを計算し、RANSACにて残ったものだけ選択
    cv::Mat inliner_mask, homography;
    std::vector<cv::KeyPoint> inliners1_keypoints, inliners2_keypoints;
    std::vector<cv::DMatch> inliners_matches, good_match;
    if (matched1_keypoints.size() < 10)
    {
        resetFirstFrame(frame);
        return;
    }
    homography = cv::findHomography(keypoint2Point(matched1_keypoints), keypoint2Point(matched2_keypoints),
                                    cv::RANSAC, threshold_ransac, inliner_mask);

    std::vector<int> inliners_idx;
    inliners_idx.clear();
    for (size_t i = 0; i < matched1_keypoints.size(); i++)
    {
        if (inliner_mask.at<uchar>(i))
        {
            inliners1_keypoints.push_back(matched1_keypoints[i]);
            inliners2_keypoints.push_back(matched2_keypoints[i]);
            int new_i = static_cast<int>(inliners1_keypoints.size());
            inliners_matches.push_back(cv::DMatch(new_i, new_i, 0));
            good_match.push_back(dmatch[i]);
            inliners_idx.push_back(i); // インデックスを整理した後にもオリジナルのインデックスを参照できるように保存
        }
    }

    if (inliners_matches.size() == 0)
    {
        resetFirstFrame(frame);
        return;
    }
    // printf("matches:%zu->%zu->%zu\n", knn_matches.size(), matched1_keypoints.size(), inliners_matches.size());

    std::map<int, LastFrame> current_frame;
    std::map<unsigned int, LastFrame> track;
    find_track.clear();
    for (size_t i = 0; i < good_match.size(); i++)
    {
        // さっきknnマッチングを行ったときのクエリ・訓練ディスクリプタのインデクスをmatched_idxとして保存した
        // その中で誤対応除去したinliners_idxに対応するインデックスを取得する
        // つまり誤対応除去されたインデックス(first_frameから見たら当然飛び飛びの値)が入っている
        int idx1 = matched1_keypoints_idx[inliners_idx[i]];
        int idx2 = matched2_keypoints_idx[inliners_idx[i]];

        // x1,y1: first_frame内でのkeypointの位置
        float x1 = inliners1_keypoints[i].pt.x; // = first_keypoint[idx1].pt.x;
        float y1 = inliners1_keypoints[i].pt.y;
        float x2 = inliners2_keypoints[i].pt.x;
        float y2 = inliners2_keypoints[i].pt.y;

        if (!last_frame.empty())
        {
            // 現在のフレームでのidx1とlast_frameの一番最後でのidx2が同じ値で設定されているか？
            // 同じ値であれば連続して追跡できていることを意味する
            auto itr = last_frame.back().find(idx1);
            if (itr != last_frame.back().end())
            {
                // 設定されている場合
                // 追跡する番号はそのまま引き継ぐ
                track.insert(std::make_pair(itr->second.track_no, LastFrame(idx1, x1, y1, idx2, x2, y2, itr->second.track_no, frame_id)));
                current_frame.insert(std::make_pair(idx2, LastFrame(idx1, x1, y1, idx2, x2, y2, itr->second.track_no, frame_id)));
                // printf("data_tra: (%d), (%d, %0.1f, %0.1f, %d, %0.1f, %0.1f, %d, %s)\n", itr->second.track_no, idx1, x1, y1, idx2, x2, y2, itr->second.track_no, frame_id.c_str());
                // printf("No.%d:[%f %f], [%f %f] [%f %f]\n",itr->second.track_no, itr->second.x1, itr->second.y1, x1, y1, x2, y2);
            }
            else
            {
                // 設定されていない場合は新規に追加
                // 追跡する番号track_noを新規のものを仕様する
                track.insert(std::make_pair(track_no, LastFrame(idx1, x1, y1, idx2, x2, y2, track_no, frame_id)));
                current_frame.insert(std::make_pair(idx2, LastFrame(idx1, x1, y1, idx2, x2, y2, track_no, frame_id)));
                // printf("data_new: (%d), (%d, %0.1f, %0.1f, %d, %0.1f, %0.1f, %d, %s)\n", track_no, idx1, x1, y1, idx2, x2, y2, track_no, frame_id.c_str());
                track_no++;
            }
        }
        else
        {
            // そもそも過去フレームが存在しないときは無条件に新規に追加
            track.insert(std::make_pair(track_no, LastFrame(idx1, x1, y1, idx2, x2, y2, track_no, frame_id)));
            current_frame.insert(std::make_pair(idx2, LastFrame(idx1, x1, y1, idx2, x2, y2, track_no, frame_id)));
            // printf("data_ini: (%d), (%d, %0.1f, %0.1f, %d, %0.1f, %0.1f, %d, %s)\n", track_no, idx1, x1, y1, idx2, x2, y2, track_no, frame_id.c_str());
            track_no++;
        }
    }
    find_track.push_back(track);
    last_frame.push_back(current_frame);

    printf("last_frame.size() = %zu\n", last_frame.size());
    printf("last_frame[0].size() = %zu\n", last_frame[0].size());
    printf("find_track.size() = %zu\n", find_track.size());
    printf("current_frame.size() = %zu\n", current_frame.size());
    printf("track.size() = %zu\n", track.size());
    printf("track_no = %d\n", track_no);

    if (track_no > threshold_track_no && last_frame.size() > 100)
    {
        last_frame.erase(last_frame.begin());
    }

    if (track_no > threshold_track_no && find_track.size() > 100)
    {
        find_track.erase(find_track.begin());
    }

    if (flag_showimage)
    {
        cv::Scalar match_line_color = cv::Scalar(255, 0, 0);
        cv::Scalar match_point_color = cv::Scalar(255, 255, 0);
        cv::drawMatches(first_frame, first_keypoint, frame, keypoint, good_match, matched_image, match_line_color, match_point_color);
        // cv::drawKeypoints(frame, inliners2_keypoints, matched_image, match_point_color);
    }

    first_frame = frame.clone();
    first_keypoint = keypoint;
    first_descriptor = descriptor;
}

void Tracker::calcFindTrack()
{
    // std::mapのキーとして特徴点ナンバーを登録したものを生成
    for (size_t i = 0; i < last_frame.size(); i++)
    {
        std::map<unsigned int, LastFrame> track;
        for (auto itr = last_frame[i].begin(); itr != last_frame[i].end(); itr++)
        {
            track.insert(std::make_pair(itr->second.track_no, itr->second));
        }
        find_track.push_back(track);
    }

    // trackの番号順に読み出し
    // printf("track_no = %d, find_track.size() = %zu\n", track_no, find_track.size());
    static unsigned int min_track_no = 0;
    bool min_setting = true;
    for (unsigned int i = min_track_no; i < this->track_no; i++)
    {
        auto itr_1 = find_track[0].find(i);

        for (size_t j = 0; j < find_track.size(); j++)
        {
            bool flag_first = false;

            if (j == 0)
            {
                // Frameが0の時に対応点の登録があれば
                if (itr_1 != find_track[0].end())
                    flag_first = true;
            }
            else
            {
                // Frameが1以上で、一個前のフレームに登録があれば
                if (itr_1 != find_track[j - 1].end())
                    flag_first = true;
            }

            // トラック番号iについて、j番目のフレーム内にありますか？
            auto itr = find_track[j].find(i);
            if (itr != find_track[j].end())
            {
                if (flag_first)
                {
                    // j-1番目のフレームにもトラック番号iが登場するなら
                    printf("TrackNo.%d, FrameNo:%d: (x1,y1)=(%f, %f)\n", itr_1->first, std::atoi(itr_1->second.frame_id.c_str()) - 1, itr_1->second.x1, itr_1->second.y1);
                    printf("TrackNo.%d, FrameNo:%d: (x2,y2)=(%f, %f)\n", itr_1->first, std::atoi(itr_1->second.frame_id.c_str()), itr_1->second.x2, itr_1->second.y2);
                    printf("TrackNo.%d, FrameNo:%d: (x1,y1)=(%f, %f)\n", itr->first, std::atoi(itr->second.frame_id.c_str()) - 1, itr->second.x1, itr->second.y1);
                    printf("TrackNo.%d, FrameNo:%d: (x2,y2)=(%f, %f)\n", itr->first, std::atoi(itr->second.frame_id.c_str()), itr->second.x2, itr->second.y2);
                }
                if (min_setting)
                {
                    min_setting = false;
                    min_track_no = itr->first;
                }
            }
            itr_1 = itr;
        }
        // printf("track_No.%d is end\n", i);
    }
}