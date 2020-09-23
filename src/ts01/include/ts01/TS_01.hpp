#include <libts01.h>
#include <math.h>

#define SAMPLING_FREQ 1000 // [Hz]
#define TS01_OPENED 15
#define ADOF 5
#define ssi_clock 16      // 16 * 100 ns
#define ssi_timeout 40000 // / 8; // 40000ns / 8ns  //30usでは短すぎる //short型の最大値が32767

#define IP_TS01 "192.168.1.100"

class Manage_TS01
{
public:
    TS01 ts01;
    TS01InputData input;
    TS01OutputData output;
};

class Manage_Encoder
{
public:
    Manage_Encoder();
    //「0〜max」から「-max/2〜max/2」に変更
    int shift_range(unsigned int val, unsigned int range);
    //アブソエンコーダのビット数の設定
    short ARM_BIT[5];
    double RQ[ADOF];

private:
    //分解能（パルス数）
    unsigned int res(int n);
};

class Manage_EyeExplorer
    : public Manage_TS01,
      public Manage_Encoder
{
public:
    Manage_EyeExplorer();
    int init_module(void);
    int cleanup_module(void);
    bool isOpened()
    {
        return flag_opened;
    }

private:
    bool flag_opened;
};