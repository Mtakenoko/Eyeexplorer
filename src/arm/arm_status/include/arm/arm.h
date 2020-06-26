#ifndef ARM_STATUS__ARM_
#define ARM_STATUS__ARM_

#define ADOF 5
#define ENDOSCOPE_LENGTH 90.0
#define ENDOSCOPE_ROOT 60.0
#define ENDOSCOPE_NEEDLE 30.0
#define M_EI 0.0
#define DEFLECTION_DERECT 0.0

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