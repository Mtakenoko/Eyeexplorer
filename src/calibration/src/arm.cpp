#include "../include/arm.hpp"

PassiveArm::PassiveArm() : Ktl::SerialMechanism<ADOF>()
{
    //土台部
    setup_link(0, Ktl::Vector<3>(9.0, 0.0, 32.0));
    setup_axis(0, Ktl::Z, 0);
    //第１リンク
    setup_link(1, Ktl::Vector<3>(200.0, 0.0, 0.0), Ktl::Vector<3>(32.32, 0.0, 17.68));
    setup_axis(1, Ktl::Y, 2);
    //第２リンク
    setup_link(2, Ktl::Vector<3>(0.0, 0.0, -200.0), Ktl::Vector<3>(54.0, 0.0, -19.0));
    setup_axis(2, Ktl::Y, 2);
    //ジンバルアルファ軸回転
    setup_link(3, Ktl::Vector<3>(107.4, 0.0, 0.0));
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
    : Ktl::SerialMechanism<ADOF>()
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
PassiveArm::PassiveArm(const T link[21]) : Ktl::SerialMechanism<ADOF>()
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

void PassiveArm::calc()
{
    // 内視鏡針部の付け根までの運動学
    Ktl::Matrix<3, 3> Escope = Ktl::Matrix<3, 3>(Ktl::Y, 180.0 / DEG) *
                               Ktl::Matrix<3, 3>(Ktl::Z, 0.0 / DEG); //現状は内視鏡の姿勢はx軸が視線方向なので画像座標と等しく（z正方向が視線方向）するための回転行列?
    endoscope_pose = this->Rr() * Escope;                            // 内視鏡姿勢行列
    Ktl::Vector<3> n = endoscope_pose.column(2);                     // 内視鏡の向き
    Ptip = this->Pr() + ENDOSCOPE_LENGTH * n;
}

void PassiveArm::calcDeflection(double rot_z, double M_EI)
{
    // 内視鏡針部の付け根までの運動学
    Ktl::Matrix<3, 3> Escope = Ktl::Matrix<3, 3>(Ktl::Y, 180.0 / DEG) *
                               Ktl::Matrix<3, 3>(Ktl::Z, 0.0 / DEG); //現状は内視鏡の姿勢はx軸が視線方向なので画像座標と等しく（z正方向が視線方向）するための回転行列?
    Ktl::Matrix<3, 3> endoscope_root_pose = this->Rr() * Escope;     // 内視鏡姿勢行列
    Ktl::Vector<3> n = endoscope_root_pose.column(2);                // 内視鏡の向き
    Ktl::Vector<3> P_root = this->Pr() + ENDOSCOPE_ROOT * n;

    // 内視鏡針部のたわみについて
    // 梁の曲げモデルを考える。x軸回りの曲げ回転とその方向を選択するz方向の回転の2つで成り立つ
    Ktl::Matrix<3, 3> Rot_r_e = Ktl::Matrix<3, 3>(Ktl::Z, rot_z);                                   // 針部の曲がる平面（力方向に水平な平面）への座標変換行列（内視鏡姿勢座標からみた）
    Ktl::Matrix<3, 3> Rot_deflect_r = Ktl::Matrix<3, 3>(Ktl::X, -M_EI * ENDOSCOPE_NEEDLE);          // 針部の曲げを表す回転行列（力に水平な面から見た）
    Ktl::Vector<3> needle_r(0., M_EI * ENDOSCOPE_NEEDLE * ENDOSCOPE_NEEDLE / 2., ENDOSCOPE_NEEDLE); // 内視鏡根本から内視鏡先端までのベクトル（力に水平な面から見た）
    endoscope_pose = endoscope_root_pose * Rot_r_e * Rot_deflect_r * Rot_r_e.inv();                 // 絶対座標系から見た、内視鏡先端の座標系
    Ptip = P_root + endoscope_root_pose * Rot_r_e * needle_r;
}