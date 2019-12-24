#include <cstdio>
#include <iostream>
#include <string>
#include <memory>
#include <string>
#include <utility>
#include <ktl.h>

#include "../include/arm/encoder.h"

#define ENC_OFFSET_FILE "enc_offset.dat" ///< エンコーダオフセットファイルのパス

ReadEncoder::ReadEncoder()
{
}
void ReadEncoder::ReadOffsetdat()
{
    std::ifstream ifs(ENC_OFFSET_FILE);

    for (int i = 0; i < ADOF; i++)
        ifs >> offset[i];
    ifs.close();
    offset *= 1.0 / DEG; //deg -> rad
}
void ReadEncoder::ResetOffset(Ktl::Vector<ADOF> qoffset)
{
    offset = qoffset;
}

Ktl::Vector<ADOF> ReadEncoder::GetOffset()
{
    return offset;
}

void ReadEncoder::SetOffset(){
    offset[0] = -50.4918;//-18.4375 + 17.677004;
    offset[1] = -75.5922;//-81.3766 + 79.902502;
    offset[2] = -19.8393;//53.5832 - 56.141618;
    offset[3] = -179.646;//177.849 - 174.721681;
    offset[4] = -124.325;//144.96 - 142.300355;
}