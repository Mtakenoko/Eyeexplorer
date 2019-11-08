#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

int encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}
std::string mat_type2encoding(int mat_type)
{
  switch (mat_type) {
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
void convert_frame_to_message(const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image & msg)
{
  // copy cv information into ros message
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = std::to_string(frame_id);
}

//オプティカルフローを可視化する。
//縦横のベクトルの強さを色に変換する。
//左：赤、右：緑、上：青、下：黄色
void visualizeFarnebackFlow(
    const cv::Mat& flow,    //オプティカルフロー CV_32FC2
    cv::Mat& visual_flow    //可視化された画像 CV_32FC3
)
{
    visual_flow = cv::Mat::zeros(flow.rows, flow.cols, CV_32FC3);
    int flow_ch = flow.channels();
    int vis_ch = visual_flow.channels();//3のはず
    for(int y = 0; y < flow.rows; y++) {
        float* psrc = (float*)(flow.data + flow.step * y);
        float* pdst = (float*)(visual_flow.data + visual_flow.step * y);
        for(int x = 0; x < flow.cols; x++) {
            float dx = psrc[0];
            float dy = psrc[1];
            float r = (dx < 0.0) ? abs(dx) : 0;
            float g = (dx > 0.0) ? dx : 0;
            float b = (dy < 0.0) ? abs(dy) : 0;
            r += (dy > 0.0) ? dy : 0;
            g += (dy > 0.0) ? dy : 0;
 
            pdst[0] = b;
            pdst[1] = g;
            pdst[2] = r;
 
            psrc += flow_ch;
            pdst += vis_ch;
        }
    }
}
            

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> g_pub;

void optical_flow(const sensor_msgs::msg::Image::SharedPtr msg, bool show_camera, rclcpp::Logger logger){

    RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
    // Convert to an OpenCV matrix by assigning the data.
    cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char *>(msg->data.data()), msg->step);

    if (msg->encoding == "rgb8") {
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    static cv::Mat cvframe, current, previous, flow, visual_flow;
    // 特徴点格納用
    std::vector<cv::Point2f> prevCorners;
    std::vector<cv::Point2f> currCorners;
    std::vector<uchar> featuresFound;
    std::vector<float> featuresErrors;

    cvframe = frame.clone();
    //グレイスケールへ変換
    cv::cvtColor(cvframe, current, CV_BGR2GRAY);

     
    //前のフレームがあれば、オプティカルフローを計算し、表示する
    if(!previous.empty()) {
         // 追跡する特徴点を求める
        //cv::goodFeaturesToTrack(previous, prevCorners, 1000, 0.1, 7);
        //cv::cornerSubPix(previous, prevCorners, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 0.03));
       
        //オプティカルフロー計算(Farneback法)
        cv::calcOpticalFlowFarneback(previous, current, flow, 0.5, 1, 15, 1, 5, 1.1, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
        //cv::calcOpticalFlowFarneback(previous, current, flow, 0.8, 10, 15, 3, 5, 1.1, 0);
        //オプティカルフロー計算(Lucas-Kanade法)
        //cv:: calcOpticalFlowPyrLK(previous, current, prevCorners, currCorners, featuresFound, featuresErrors, cv::Size(40, 40));
        
        //3*3の小領域でのフローの方向の分散
        cv::Rect mini_rect;
        cv::Scalar mean, stddev;
        cv::Point2f zero;
        for(int i=1; i<current.cols-1;i++){
          for(int j=1; j<current.rows-1; j++){
              mini_rect = cv::Rect(i-1, j-1, 3, 3);
              cv::Mat mini_flow(flow, mini_rect);
              cv::meanStdDev(mini_flow, mean, stddev);
              if(stddev[0] >= 0.000){ //0.0001
                //flow.at<int>(i, j) = 0, 0);
              }
          }
        }
        
        ////フローを可視化
        std::vector<cv::Point2f> prev_pts;
        std::vector<cv::Point2f> next_pts;
        cv::Size flowSize(100,100); //ベクトルの数
        cv::Point2f center = cv::Point(current.cols/2., current.rows/2.);
        for(int i=0; i<flowSize.width; ++i) {
            for(int j=0; j<flowSize.width; ++j) {
                cv::Point2f p(i*float(current.cols)/(flowSize.width-1), j*float(current.rows)/(flowSize.height-1));
                prev_pts.push_back((p-center)*0.95f+center);
            }
        }
        std::vector<cv::Point2f>::const_iterator p = prev_pts.begin();
        for(; p!=prev_pts.end(); ++p) {
            cv::Point2f& fxy = flow.at<cv::Point2f>(p->y, p->x);
            if((pow(fxy.x,2)+pow(fxy.y,2)) < 1){
              fxy.x = 0.0;
              fxy.y = 0.0;
            }
            cv::line(cvframe, *p, *p+fxy*12, cv::Scalar(255, 0, 0),1);
        }
        ////
        /*for (int i = 0; i < featuresFound.size(); i++) {
            if (featuresFound[0] == 0 || featuresErrors[i] > 100) {
                continue;
            }

            cv::Point p1 = cv::Point((int)prevCorners[i].x, (int)prevCorners[i].y);
            cv::Point p2 = cv::Point((int)currCorners[i].x, (int)currCorners[i].y);
            cv::line(current, p1, p2, cv::Scalar(255, 0, 0), 2);
        }*/

        //表示
        //cv::imshow("flow", visual_flow);
        cv::imshow("cvframe", cvframe);
        //cv::imshow("current", current);
        //cv::imshow("previous", previous);
        cv::waitKey(1);
    }
        
    //前のフレームを保存
    previous = current.clone();


    //Publish Image
    RCLCPP_INFO(logger, "Publishing image #%s", msg->header.frame_id.c_str());
    auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();

    convert_frame_to_message(cvframe, atoi(msg->header.frame_id.c_str()), *msg_pub);  //cv → msg
    g_pub->publish(std::move(msg_pub)); 
}

int main(int argc, char * argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    //std::string topic_sub("preprocessed_image");   
    std::string topic_sub("endoscope_image");   
    std::string topic_pub("feature_point");
    bool show_camera = false;

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("feature_extract");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    auto callback = [show_camera, &node](const sensor_msgs::msg::Image::SharedPtr msg_sub){
        optical_flow(msg_sub, show_camera, node->get_logger());
    };    

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
    g_pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos); // Create the image publisher with our custom QoS profile.
    
    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(topic_sub, qos, callback);  // Initialize a subscriber that will receive the ROS Image message to be displayed.


    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}