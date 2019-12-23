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
    offset[0] = 8.30911;
    offset[1] = -7.06148;
    offset[2] = 46.2615;
    offset[3] = 146.826;
    offset[4] = -39.8613;
}