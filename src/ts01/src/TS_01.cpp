#include "../include/ts01/TS_01.hpp"

Manage_Encoder::Manage_Encoder()
{
  //アブソエンコーダのビット数の設定
  ARM_BIT[0] = 20;
  ARM_BIT[1] = 19;
  ARM_BIT[2] = 19;
  ARM_BIT[3] = 18;
  ARM_BIT[4] = 18;

  // カウント方向込
  RQ[0] = -2 * M_PI / res(ARM_BIT[0]);
  RQ[1] = -2 * M_PI / res(ARM_BIT[1]);
  RQ[2] = 2 * M_PI / res(ARM_BIT[2]);
  RQ[3] = 2 * M_PI / res(ARM_BIT[3]);
  RQ[4] = 2 * M_PI / res(ARM_BIT[4]);
}

//分解能（パルス数）
unsigned int Manage_Encoder::res(int n)
{
  //16進数0xffffffff　=>　2進数2＾32
  //右シフトで分解能に変換
  //例（18bitのエンコーダ）
  //11...1=>32個
  //右に（32-18）bitシフト
  //00..0->14個　11..1->18個　=>2の18乗
  return 0xffffffff >> (32 - n);
}

//「0〜max」から「-max/2〜max/2」に変更
int Manage_Encoder::shift_range(unsigned int val, unsigned int range)
{
  if (val < range / 2) //+30000)
    return (int)val;
  else
    return (int)val - (int)range;
}

Manage_EyeExplorer::Manage_EyeExplorer()
    : flag_opened(false)
{
}

/*******************************************************************
 *     init_module
 ****************************************************************** */
int Manage_EyeExplorer::init_module(void)
{
  int ts01_status = ts01.open(IP_TS01);
  if (ts01_status != TS01_UNOPENED)
  {
    //--- SSI -----------------------------------------------
    for (int j = 0; j < ADOF; j++)
    {
      ts01.setup_ssi(j, ssi_clock, ARM_BIT[j] + 1, ssi_timeout);
    }
    //各 dizital out channel に pulse 生成準備-----------------------------------------
    //true:パルス入力　false:デジタル入力（非パルス）
    // ts01.set_dout_mode(0, true);  //パルス
    // ts01.set_dout_mode(1, true);  //パルス
    // ts01.set_dout_mode(2, true);  //パルス
    // ts01.set_dout_mode(3, false); //デジタル入力（非パルス）
    // ts01.set_dout_mode(4, true);  //パルス
    // ts01.set_dout_mode(5, false); //デジタル入力（非パルス）
    // ts01.set_dout_mode(6, true);  //パルス
    // ts01.set_dout_mode(7, false); //デジタル入力（非パルス）

    //--- counter ---------------------------
    ts01.set_count(0);
    ts01.start_count();

    //--- AO ---------------------------
    for (int j = 0; j < TS01_AO_CH_NUM; j++)
      output.u[j] = 5.0;
    ts01.write_data(&output);

    // TS-01がopen
    ts01.start_sampling(SAMPLING_FREQ);

    // Openフラグ
    this->flag_opened = true;
  }

  return ts01_status;
}

/*******************************************************************
 *     cleanup_module
 *******************************************************************/
int Manage_EyeExplorer::cleanup_module(void)
{
  if (!this->isOpened())
  {
    rt_print("control module has not opened.\n");
    return 1;
  }

  ts01.stop_sampling();
  for (int j = 0; j < TS01_DO_CH_NUM; j++)
  {
    output.dout[j] = false;
  }
  for (int j = 0; j < TS01_AO_CH_NUM; j++)
  {
    output.u[j] = 0.0;
  }
  ts01.write_data(&output);
  ts01.close();
  this->flag_opened = false;
  rt_print("control module has been removed.\n");
  return 1;
}