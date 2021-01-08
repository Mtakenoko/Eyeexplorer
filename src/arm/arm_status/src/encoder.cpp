#include <iostream>
#include <ktl.h>

#include "../include/arm/encoder.h"

#define ENC_OFFSET_FILE "/home/takeyama/workspace/ros2_eyeexplorer/src/arm/arm_status/offset/enc_offset.txt"     ///< エンコーダオフセットファイルのパス
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
    offset[0] += 0.802174;
    offset[1] += 1.26729;
    offset[2] += 0.139956;
    offset[3] += 2.743183;
    offset[4] += -2.537218;
}