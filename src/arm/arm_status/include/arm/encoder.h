#ifndef __ENCODER_
#define __ENCODER_

#include <ktl.h>

#define ADOF 5

class ReadEncoder
{
public:
  Ktl::Vector<ADOF> offset;

private:

public:
  explicit ReadEncoder();
  void ReadOffsetdat();
  void WriteOffsetdat();
  void ResetOffset(Ktl::Vector<ADOF> qoffset);
  Ktl::Vector<ADOF> GetOffset();
  void SetOffset();
};
#endif