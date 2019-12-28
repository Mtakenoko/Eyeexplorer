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
    for (int i = 0; i < ADOF; i++)
        offset[i] = 0.0;
}
void ReadEncoder::ReadOffsetdat()
{
    std::ifstream ifs(ENC_OFFSET_FILE);

    for (int i = 0; i < ADOF; i++)
    {
        ifs >> offset[i];
        offset[i] *= 1.0 / DEG; //deg -> rad
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
    offset[0] = -0.599469; //-46.2371; //-18.4375 + 17.677004;
    offset[1] = -1.342200; //-75.242;  //-81.3766 + 79.902502;
    offset[2] = -0.252688; //-18.9048; //53.5832 - 56.141618;
    offset[3] = 2.498481;  //54.4841;  //177.849 - 174.721681;
    offset[4] = -1.277739; //76.7439;  //144.96 - 142.300355;
}