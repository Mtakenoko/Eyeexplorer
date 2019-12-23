#ifndef __ARM_
#define __ARM_

#include <ktl.h>

#define ADOF 5
#define ENDOSCOPE_LENGTH 0.0

class PassiveArm : public Ktl::SerialMechanism<ADOF>
{
public:
  PassiveArm();
  int Getversion();
  void Setversion(int val);
  Ktl::Matrix<3, 3> Tw; //gimbal の設置角度

  void inverse_kinematics() override;

private:
  int arm_version;
};

#endif