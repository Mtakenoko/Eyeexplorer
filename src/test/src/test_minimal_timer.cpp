#include <rclcpp/rclcpp.hpp>
#include <chrono>

rclcpp::Node::SharedPtr node = nullptr;

int main(int argc, char * argv[]){
  using namespace std::chrono_literals;

  rclcpp::init(argc,argv);
  node = rclcpp::Node::make_shared("minimal_node");

  rclcpp::WallRate loop_rate(500ms);
  for(int i=0 ; i<3 ; i++){
    RCLCPP_INFO(node->get_logger(), "loop:%d",i);
    loop_rate.sleep();
  }

  auto timer1 = node->create_wall_timer(
    1s,
    [](){
      RCLCPP_INFO(node->get_logger(),"node_loop");
    }
  );
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}