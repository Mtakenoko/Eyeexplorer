#include <ktl.h>
#include "../include/arm/arm.h"

PassiveArm::PassiveArm()
{
  double offsetAngle = 0.0 / DEG;
  Tw.setRot(Ktl::Y, offsetAngle);

  setup_link(0, Ktl::Vector<3>(0.0, 0.0, 28.0));
  setup_axis(0, Ktl::Z, 0);

  setup_link(1, Ktl::Vector<3>(200.0, 0.0, 0.0));
  setup_axis(1, Ktl::Y, 2);

  //setup_link(2, Ktl::Vector<3>( 0.0, 0.0, -200.0 ), Tw*Ktl::Vector<3>(25.0,0.0, 34.0) );
  setup_link(2, Ktl::Vector<3>(0.0, 0.0, -200.0), Ktl::Vector<3>(39.82, 0.0, -22.39));
  setup_axis(2, Ktl::Y, 2);

  setup_link(3, Tw * Ktl::Vector<3>(40.0, 0.0, 0.0));
  setup_axis(3, Tw * Ktl::ex(), 0);

  setup_link(4, 0.0, Ktl::Z, Tw * Ktl::Vector<3>(30.2, 0.0, 0.0));
  setup_axis(4, Tw * Ktl::ey(), 0);
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
