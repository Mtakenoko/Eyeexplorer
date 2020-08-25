#include <cstdio>
#include <iostream>
#include <string>
#include <memory>
#include <string>
#include <utility>
#include <ktl.h>

#include "../include/arm/encoder.h"

#define ENC_OFFSET_FILE "/home/takeyama/workspace/ros2_eyeexplorer/src/arm/arm_status/offset/enc_offset.dat"     ///< エンコーダオフセットファイルのパス
#define Calib_OFFSET_FILE "/home/takeyama/workspace/ros2_eyeexplorer/src/arm/arm_status/offset/calib_offset.txt" ///< エンコーダオフセットファイルのパス

ReadEncoder::ReadEncoder()
{
    for (int i = 0; i < ADOF; i++)
        offset[i] = 0.0;
}
void ReadEncoder::ReadOffsetdat()
{
    std::ifstream ifs(ENC_OFFSET_FILE);
    double data[ADOF];
    for (int i = 0; i < ADOF; i++)
    {
        ifs >> data[i];
        data[i] *= 1.0 / DEG; //deg -> rad
        offset[i] += data[i];
    }
    ifs.close();
}

void ReadEncoder::ReadCalibOffsetdat()
{
    std::ifstream ifs(Calib_OFFSET_FILE);
    double data[ADOF];
    for (int i = 0; i < ADOF; i++)
    {
        ifs >> data[i];
        offset[i] += data[i];
    }
    ifs.close();
}

void ReadEncoder::WriteOffsetdat()
{
    std::ofstream ofs(ENC_OFFSET_FILE);

    for (int i = 0; i < ADOF; i++)
    {
        ofs << offset[i] * DEG;
    }
    ofs.close();
}

void ReadEncoder::ResetOffset(Ktl::Vector<ADOF> qoffset)
{
    offset = qoffset;
}

Ktl::Vector<ADOF> ReadEncoder::GetOffset()
{
    offset *= 1.0 / DEG;
    return offset;
}

void ReadEncoder::SetOffset()
{
    offset[0] += 0.695877 + 0.170817 - 0.135620 + 0.0197 - 0.03;
    offset[1] += 1.313390 - 0.0039;
    offset[2] += 0.250914 + 0.224633 - 0.368491 + 0.0039 - 0.074;
    offset[3] += -2.325260 - 0.054457 + 4.9320 + 0.1909;
    offset[4] += 1.346482 - 3.9817 + 0.2939;
}