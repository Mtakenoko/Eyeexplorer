#include <ktl.h>
#include "../include/arm/arm.h"

#define KV Ktl::Vector<3>
#define ZERO Ktl::Vector<3>(.0, .0, .0)
#define X(num) Ktl::Vector<3>(num, 0.0, 0.0)
#define Y(num) Ktl::Vector<3>(0.0, num, 0.0)
#define Z(num) Ktl::Vector<3>(0.0, 0.0, num)

PassiveArm::PassiveArm()
{
    double offsetAngle = 0.0 / DEG;
    Tw.setRot(Ktl::Y, offsetAngle);

    setup_link(0, Ktl::Vector<3>(0.0, 0.0, 32.0), Ktl::Vector<3>(9.0, 0.0, 0.0));
    setup_axis(0, Ktl::Z, 0);

    setup_link(1, Ktl::Vector<3>(200.0, 0.0, 0.0), Ktl::Vector<3>(18.18, 0.0, 3.54));
    setup_axis(1, Ktl::Y, 2);

    setup_link(2, Ktl::Vector<3>(0.0, 0.0, -200.0), Ktl::Vector<3>(56.54, 0.5, -4.86));
    setup_axis(2, Ktl::Y, 2);

    setup_link(3, Tw * Ktl::Vector<3>(94.0, 0.0, 0.0));
    setup_axis(3, Tw * Ktl::ex(), 0);

    setup_link(4, 0.0, Ktl::Z);
    setup_axis(4, Tw * Ktl::ey(), 0);

    setup_link(5, 0.0, Ktl::X);
    setup_axis(5, -Tw * Ktl::ez(), 0);
}
int PassiveArm::Getversion(){
    return arm_version;
}
void PassiveArm::Setversion(int val){
    arm_version = val;
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