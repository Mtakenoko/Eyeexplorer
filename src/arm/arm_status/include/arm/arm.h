#ifndef ARM_STATUS__ARM_
#define ARM_STATUS__ARM_

#define ADOF 5
#define ENDOSCOPE_LENGTH 90.0

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