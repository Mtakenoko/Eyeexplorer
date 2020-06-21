#ifndef ARM_STATUS_HPP_
#define ARM_STATUS_HPP_

#include <ktl.h>

#define ADOF 5
#define ENDOSCOPE_LENGTH 90.0

class PassiveArm : public Ktl::SerialMechanism<ADOF>
{
public:
  PassiveArm();
  template <typename T>
  PassiveArm(const T &link0_x, const T &link0_y, const T &link0_z,
             const T &link1_x, const T &link1_y, const T &link1_z,
             const T &link2_x, const T &link2_y, const T &link2_z,
             const T &link3_x, const T &link3_y, const T &link3_z,
             const T &link4_x, const T &link4_y, const T &link4_z,
             const T &link5_x, const T &link5_y, const T &link5_z);

  template <typename T>
  PassiveArm(const T *link);
};

#endif