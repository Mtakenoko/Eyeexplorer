#ifndef ARM_STATUS__ARM_
#define ARM_STATUS__ARM_

#include <ktl.h>

#define ADOF 5
#define ENDOSCOPE_LENGTH 110.0

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
  PassiveArm(const T link[21]);
  int Getversion();
  void Setversion(int val);
  Ktl::Matrix<3, 3> Tw; //gimbal の設置角度

  void inverse_kinematics() override;

private:
  int arm_version;
};

PassiveArm::PassiveArm()
{
  //土台部
  setup_link(0, Ktl::Vector<3>(9.0, 0.0, 32.0));
  setup_axis(0, Ktl::Z, 0);
  //第１リンク
  setup_link(1, Ktl::Vector<3>(200.0, 0.0, 0.0), Ktl::Vector<3>(32.32, 0.0, 17.68));
  setup_axis(1, Ktl::Y, 2);
  //第２リンク
  setup_link(2, Ktl::Vector<3>(0.0, 0.0, -200.0), Ktl::Vector<3>(41.5, 0.5, -19.0));
  setup_axis(2, Ktl::Y, 2);
  //ジンバルアルファ軸回転
  setup_link(3, Ktl::Vector<3>(94.9, 0.0, 0.0));
  setup_axis(3, Ktl::X, 0);
  //ジンバルベータ回転
  setup_link(4, 0.0, Ktl::Y);
  setup_axis(4, Ktl::Y, 0);
}

template <typename T>
PassiveArm::PassiveArm(const T &link0_x, const T &link0_y, const T &link0_z,
                       const T &link1_x, const T &link1_y, const T &link1_z,
                       const T &link2_x, const T &link2_y, const T &link2_z,
                       const T &link3_x, const T &link3_y, const T &link3_z,
                       const T &link4_x, const T &link4_y, const T &link4_z,
                       const T &link5_x, const T &link5_y, const T &link5_z)
{
  // 土台部
  setup_link(0, Ktl::Vector<3>(link0_x, link0_y, link0_z));
  setup_axis(0, Ktl::Z, 0);
  // 第１リンク
  setup_link(1, Ktl::Vector<3>(link2_x, link2_y, link2_z), Ktl::Vector<3>(link1_x, link1_y, link1_z));
  setup_axis(1, Ktl::Y, 2);
  // 第２リンク
  setup_link(2, Ktl::Vector<3>(link4_x, link4_y, link4_z), Ktl::Vector<3>(link3_x, link3_y, link3_z));
  setup_axis(2, Ktl::Y, 2);
  // ジンバルアルファ軸回転
  setup_link(3, Ktl::Vector<3>(link5_x, link5_y, link5_z));
  setup_axis(3, Ktl::X, 0);
  // ジンバルベータ回転
  setup_link(4, 0.0, Ktl::Y);
  setup_axis(4, Ktl::Y, 0);
}

template <typename T>
PassiveArm::PassiveArm(const T link[21])
{
  //土台部
  setup_link(0, Ktl::Vector<3>(link[0], link[1], link[2]));
  setup_axis(0, Ktl::Z, 0);
  //第１リンク
  setup_link(1, Ktl::Vector<3>(link[6], link[7], link[8]), Ktl::Vector<3>(link[3], link[4], link[5]));
  setup_axis(1, Ktl::Y, 2);
  //第２リンク
  setup_link(2, Ktl::Vector<3>(link[12], link[13], link[14]), Ktl::Vector<3>(link[9], link[10], link[11]));
  setup_axis(2, Ktl::Y, 2);
  //ジンバルアルファ軸回転
  setup_link(3, Ktl::Vector<3>(link[15], link[16], link[17]));
  setup_axis(3, Ktl::X, 0);
  //ジンバルベータ回転
  setup_link(4, 0.0, Ktl::Y);
  setup_axis(4, Ktl::Y, 0);
  // 内視鏡先端
  setup_link(5, Ktl::Vector<3>(link[18], link[19], link[20]));
  setup_axis(5, Ktl::Z, 0);
}
void PassiveArm::inverse_kinematics()
{

  const Ktl::Vector<3> Pref = P;
  const Ktl::Matrix<3, 3> Rref = R;
  Ktl::Vector<6> dr;

  for (int n = 0; n < 100; n++)
  {
    forward_kinematics();

    Ktl::Vector<3> w = R.column(0) * Rref.column(0) + R.column(1) * Rref.column(1) + R.column(2) * Rref.column(2);
    //下記と等価
    /*
      Vector<3> w     = 
      R.column(0) * (Rref.column(0)-R.column(0))+
      R.column(1) * (Rref.column(1)-R.column(1))+
      R.column(2) * (Rref.column(2)-R.column(2));
    */
    dr.insert(0, Pref - P);
    dr.insert(3, w);
    dr.insert(3, 0.5 * w);

    Ktl::Matrix<6, ADOF> Ja = J();
    Ktl::Matrix<ADOF, 6> Ji = Ja.inv();

    Ktl::Vector<ADOF> dq = Ji * dr;

    q += dq;

    if (n > 10)
      printf("Ktl::SerialMechanism::inverse_kinematics : n=%d\n", n);
    if (dr.abs() < 0.001)
      break;
  }
}
#endif