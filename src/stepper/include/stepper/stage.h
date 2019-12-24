#ifndef __STAGE__
#define __STAGE__

#include "def.h"

using namespace Ktl;

/**
 * @brief スピード制御。
 * 目標速度に向けた加速を制御する。
 */
class SpeedOperator{
  public:
    bool isActivated()const{return flagActivated;}

    /**
     * @brief 0から立ち上げ軌道
     * 
     * @param time タイムスタンプ
     * @param mval 速度
     * @note ジャーク最小にするとTS-01からのクロックが何故か遅れる。
     */
    void turnUp(double time, double mval){
      vel.create(mval, 0.0, shifting_time, time, Ktl::TRJ_LINEAR); 
      flag_stopping = false;
      flag_vtv = true;
      flagActivated = true;
    }

    /**
     * @brief 0へ立ち下げ軌道
     * 
     * @param time タイムスタンプ
     */
    void turnDown(double time){
      vel.create(0.0, vel.get(time), shifting_time, time, Ktl::TRJ_LINEAR);
      flag_stopping = true;
      flag_vtv = true;
    }

    /**
     * @brief 
     * 
     * @param time タイムスタンプ
     * @return double 
     */
    double value(double time){
      if( flag_stopping && !flag_vtv ){
        flagActivated = false;
        flag_stopping = false;
      }
      return vel.get(time, &flag_vtv);
    }

  private:
    const double shifting_time = 0.3;
    bool flag_vtv = false;  ///< 過渡状態
    bool flag_stopping = false;  
    Ktl::PPTrajectory<double> vel;
    bool flagActivated = false;
};


enum HomePositioningState{  ///< ホームポジショニングモード動作状態
  HOME_POS_STATE_IDLE,                ///< 待機中
  HOME_POS_CLEAR_ALARM,               ///< アラーム解除中
  HOME_POS_STATE_MOVING_TO_STOP_POS,  ///< 基準位置へ移動中
  HOME_POS_STATE_MOVING_TO_HOME_POS,  ///< ホーム位置へ移動中
  HOME_POS_STATE_NUM
};


class Controller;

/**
 * @brief ステージ。
 * ステージに関連するデータの管理と、制御に関する計算を行う。
 */
class Stage : public Ktl::SerialMechanism<3>{
  enum Mode{
    MODE_IDLE,              ///< 待機モード
    MODE_CARTESIAN_MOTION,  ///< 直交座標系移動モード
    MODE_VIEW_MOTION,       ///< 視点移動モード
    MODE_HOME_POS,          ///< ホームポジショニングモード
    MODE_AUTO_HOME_POS,     ///< オートホームポジショニングモード
    MODE_VISUAL_SERVO,      ///< ビジュアルサーボモード
    MODE_MANUAL_CONTROL,    ///< フットコントローラー等での手動制御モード
    MODE_SAFETY_CONTROL,    ///< 安全制御
    MODE_FORCED_STOP,       ///< 強制停止モード
    MODE_NUM
  };

  public:
    Stage();
    void SetAlarmState(Ktl::Direction motor, bool alarmState);
    bool GetAlarmState(Ktl::Direction motor);
    void SetLimitSensorState(Ktl::Direction axis, bool limitState);
    double GetControlSpeed(Ktl::Direction motor);
    bool GetControlDirection(Ktl::Direction motor);
    double GetAlarmClearSignal();
    Ktl::Vector<3> GetPdref() const;
    OperationMode GetOperationMode() const;
    void Move(MotionMode motion, int opDir, int direction, double speed, double time);
    void Stop(double time);
    void ClearAlarm(double time);
    void StartHomePositioning();
    void StartVisualServo();
    void ManualControl();
    void StartSafetyControl(Controller* ctrl, double time);
    void StopManualControl();
    void ForcedStop();
    void UpdateControlParameter(Controller* ctrl, double time);
    bool CountClear();  /// @todo 関数名再検討
    bool IsAtLimit(Ktl::Direction axis, double position) const;

    Ktl::Vector<3> Pdref;         ///< ステージの速度指令値
    
  private:
    Ktl::Vector<3> GetViewMotionVector(Ktl::Vector<3> Pdv, Controller* ctrl);
    void ConvertToDropOffPreventingSpeed(Ktl::Vector<3> &speed, Ktl::Vector<3> endoscopePose);
    bool IsAtHome();
    bool IsMovingOverLimit(Ktl::Vector<3> speed);

    Mode mode;                    ///< モード
    HomePositioningState state;   ///< ホームポジショニング状態
    double timeAtAlarmClearing;   ///< アラームクリア開始時タイムスタンプ
    bool alarm[3];                ///< 各モーターのアラーム状態
    bool limitSensor[3];          ///< 各軸のリミットセンサー状態
    SpeedOperator speedOperator;  ///< スピード制御
    Ktl::Vector<3> nextSpeed;     ///< ステージの次の速度指令値

    bool flagAlarmClear;
    bool countClear;              ///< カウント値をクリアすることを示すフラグ
};

#endif
