#include <stdio.h>
#include <ros/ros.h>                      

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2\video.hpp>


static const std::string OPENCV_WINDOW = "Image window";
static bool IsFirst = true;

std::vector<cv::KeyPoint> g_keypoints1, g_keypoints2;

std::vector<cv::DMatch> matching(cv::Mat dst1, cv::Mat dst2){
    /*
	// 画像ファイルを読み込み、レンズの歪み補正
	cv::undistort(cv::imread("./images/starDestroyer_0001.png"), dst1, cameraMatrix, distCoeffs);
	cv::undistort(cv::imread("./images/starDestroyer_0050.png"), dst2, cameraMatrix, distCoeffs);
	*/

	// AKAZE特徴抽出
	cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0001f); // 検出器（自分で設定）
	cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = cv::AKAZE::create();// 特徴量
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce"); // 対応点探索方法の設定

	std::vector<cv::Scalar> keyPointsColor1, keyPointsColor2;
	cv::Mat descriptor1, descriptor2;

	detector->detect(dst1, g_keypoints1);
	descriptorExtractor->compute(dst1, g_keypoints1, descriptor1);

	detector->detect(dst2, g_keypoints2);
	descriptorExtractor->compute(dst2, g_keypoints2, descriptor2);

	//対応点の探索
	std::vector<cv::DMatch> dmatch;
	std::vector<cv::DMatch> dmatch12, dmatch21;
	std::vector<cv::Point2f> match_point1,match_point2;

	matcher->match(descriptor1, descriptor2, dmatch12); //dst1 -> dst2
	matcher->match(descriptor2, descriptor1, dmatch21); //dst2 -> dst1

	//dst1 -> dst2 と dst2 -> dst1の結果が一致しているか検証
	int good_count = 0;
	for (int i = 0; i < dmatch12.size(); i++) {		
		cv::DMatch m12 = dmatch12[i];
		cv::DMatch m21 = dmatch21[m12.trainIdx];

		if (m21.trainIdx == m12.queryIdx) {// 一致しているものだけを抜粋
			dmatch.push_back(m12);
			match_point1.push_back(keypoints1[dmatch12[i].queryIdx].pt);
			match_point2.push_back(keypoints2[dmatch12[i].trainIdx].pt);
			good_count++;
		}
	}

	//ホモグラフィ行列推定
	const int MIN_MATCH_COUNT = 10;
	if (good_count > MIN_MATCH_COUNT) { //十分対応点が見つかるならば
		cv::Mat masks;
		cv::Mat H = cv::findHomography(match_point1, match_point2, masks, cv::RANSAC, 50);

		//RANSACで使われた対応点のみ抽出
		std::vector<cv::DMatch> inlinerMatch;
		for (int i = 0; i < masks.rows; i++) {
			uchar *inliner = masks.ptr<uchar>(i);
			if (inliner[0] == 1) {
				inlinerMatch.push_back(dmatch[i]);
			}
		}    
    	//インライアの対応点のみ表示
		cv::Mat drawMatch_inliner;
		cv::drawMatches(dst1, g_keypoints1, dst2, g_keypoints2, inlinerMatch, drawMatch_inliner);
	} 
	else {
		printf("Not enough matches are found\n");
		//アウトライア未除去の表示
		cv::Mat matchImage; 
		cv::drawMatches(dst1, g_keypoints1, dst2, g_keypoints2, dmatch, matchImage);
	}
    return dmatch;
}

cv::Mat reconstruct(std::vector<cv::DMatch> dmatch){
    if(dmatch.size() < 5){
        return;
    }
    std::vector<cv::Point2d> p1;
	std::vector<cv::Point2d> p2;

	//対応付いた特徴点の取り出しと焦点距離1.0のときの座標に変換
	for (size_t i = 0; i < dmatch.size(); i++) {
		cv::Mat ip(3, 1, CV_64FC1);
		cv::Point2d p;

		ip.at<double>(0) = g_keypoints1[dmatch[i].queryIdx].pt.x;
		ip.at<double>(1) = g_keypoints1[dmatch[i].queryIdx].pt.y;
		ip.at<double>(2) = 1.0;

		ip = cameraMatrix.inv()*ip;
		p.x = ip.at<double>(0);
		p.y = ip.at<double>(1);
		p1.push_back(p);

		ip.at<double>(0) = g_keypoints2[dmatch[i].trainIdx].pt.x;
		ip.at<double>(1) = g_keypoints2[dmatch[i].trainIdx].pt.y;
		ip.at<double>(2) = 1.0;

		ip = cameraMatrix.inv()*ip;
		p.x = ip.at<double>(0);
		p.y = ip.at<double>(1);
		p2.push_back(p);
	}

	cv::Mat mask; //RANSACの結果を保持するためのマスク
	cv::Mat essentialMat = cv::findEssentialMat(p1, p2, 1.0, cv::Point2f(0, 0), cv::RANSAC, 0.9999, 0.003, mask);

	cv::Mat r, t;
	cv::recoverPose(essentialMat, p1, p2, r, t);

	//正規化座標系で計算しているのでProjection matrix = Extrinsic camera parameter matrix
	cv::Mat prjMat1, prjMat2;
	prjMat1 = cv::Mat::eye(3, 4, CV_64FC1); //片方は回転、並進ともに0
	prjMat2 = cv::Mat(3, 4, CV_64FC1);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			prjMat2.at<double>(i, j) = r.at<double>(i, j);
		}
	}
	prjMat2.at<double>(0, 3) = t.at<double>(0);
	prjMat2.at<double>(1, 3) = t.at<double>(1);
	prjMat2.at<double>(2, 3) = t.at<double>(2);

	cv::Mat point3D;//三角測量による三次元位置の推定
	cv::triangulatePoints(prjMat1, prjMat2, p1, p2, point3D);
    return point3D;
}

class Reconstruction
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
    
  public:
    Reconstruction()
        : it(nh)
    {
        image_sub = it.subscribe("/image_converter/output_video", 1, &Reconstruction::imageCallback, this);
        image_pub = it.advertise("/image_converter/output_video_HighPass", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }
    
    ~Reconstruction()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }
        
    // コールバック関数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        cv::Mat ROI_image;
        cv::Mat point3d;
        try
        {
        //ROIのかかった画像をcv::Matのフォーマットに変換
        ROI_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;  // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        
        //比較用画像の用意
        cv::Mat dst1, dst2;
        static int frame = 0;
        int lag_time = 20;
        if (frame % lag_time == 10 && frame % lag_time != 0) {
			dst1 = ROI_image.clone();
		}
		else if (frame % lag_time == 0) {
			dst2 = ROI_image.clone();
            cv::Mat matchimg = matching(dst1, dst2);    //特徴量抽出とマッチング
            point3d = matchimg(matchimg);   //マッチング点を3次元復元（カメラの位置座標必要）
        }
        
        //パブリッシュ
        sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", point3d).toImageMsg();
        image_pub.publish(pub_msg);
    }        
};       

int main(int argc, char **argv) {
    ros::init(argc, argv, "3d_reconstruction"); 
    Reconstruction reconst;
    ros::spin();      
}

