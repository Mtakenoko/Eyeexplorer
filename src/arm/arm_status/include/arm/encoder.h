#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include <ktl.h>

#define ADOF 5
#define ENDOSCOPE_LENGTH 110.0

class ReadEncoder{
  public:
    explicit ReadEncoder();
    bool transformRotMatToQuaternion(
      float &qx, float &qy, float &qz, float &qw,
      float m11, float m12, float m13,
      float m21, float m22, float m23,
      float m31, float m32, float m33);
    void QuaternionToEulerAngles(
      double q0, double q1, double q2, double q3,
      double& roll, double& pitch, double& yaw);
    void EncoderOffset(Ktl::Vector<ADOF> &offset);
};