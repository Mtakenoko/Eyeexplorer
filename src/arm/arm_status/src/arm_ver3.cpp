#include <ktl.h>
#include "../include/arm/arm.h"

PassiveArm::PassiveArm()
{
  //土台部
  setup_link(0, Ktl::Vector<3>(9.0, 0.0, 32.0));
  setup_axis(0, Ktl::Z, 0);
  //第１リンク
  setup_link(1, Ktl::Vector<3>(200.0, 0.0, 0.0), Ktl::Vector<3>(32.32, 0.0, 17.68));
  setup_axis(1, Ktl::Y, 2);
  //第２リンク
  setup_link(2, Ktl::Vector<3>(0.0, 0.0, -200.0), Ktl::Vector<3>(54.0, 0.5, -19.0));
  setup_axis(2, Ktl::Y, 2);
  //ジンバルアルファ軸回転
  setup_link(3, Ktl::Vector<3>(107.4, 0.0, 0.0));
  setup_axis(3, Ktl::X, 0);
  //ジンバルベータ回転
  setup_link(4, 0.0, Ktl::Y);
  setup_axis(4, Ktl::Y, 0);
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