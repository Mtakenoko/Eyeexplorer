#include <ktl.h>

#ifndef __ARM_
#define __ARM_

#define ADOF 5
#define ENDOSCOPE_LENGTH 110.0

class Gimbal : public Ktl::SerialMechanism<3>{
 public:

  Gimbal();
  void forward_kinematics(const Ktl::Vector<3>& P0=Ktl::o(),
			  const Ktl::Matrix<3,3>& R0=Ktl::I());
};

class PassiveArm :public Ktl::SerialMechanism<ADOF>{
  public:
    PassiveArm();
    Ktl::Matrix<3,3> Tw; //gimbal の設置角度

    void inverse_kinematics()override;
};

#endif