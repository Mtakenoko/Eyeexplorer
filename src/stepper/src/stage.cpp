#include <ktl.h>
#include "controller.h"
#include "../include/stepper/stage.h"

#define STAGE_SPEED_COEF  5000  ///< ステージ速度係数

#define ALARM_SIGNAL_VALUE_ON   5.0  ///< アラーム解除信号ON時の値
#define ALARM_SIGNAL_VALUE_OFF  0.0  ///< アラーム解除信号OFF時の値

#define ALARM_CLEAR_SIGNAL_PERIOD  (50 * 1e-3)

#define HOME_POSITION_X  75.0
#define HOME_POSITION_Y  75.0
#define HOME_POSITION_Z  75.0
#define MIN_LIMIT_X  (-73.0)
#define MIN_LIMIT_Y  (-73.0)
#define MIN_LIMIT_Z  (-73.0)
#define MAX_LIMIT_X  73.0
#define MAX_LIMIT_Y  73.0
#define MAX_LIMIT_Z  73.0

#define VISUAL_SERVO_DEADBAND_THRESH   0.1
#define MANUAL_DEADBAND_THRESH  0.5
#define SLOW_DOWN_THRESHOLD  0.1


static const Ktl::Vector<3> homePos = {HOME_POSITION_X, HOME_POSITION_Y, HOME_POSITION_Z};
static const Ktl::Vector<3> minLim = {MIN_LIMIT_X, MIN_LIMIT_Y, MIN_LIMIT_Z};
static const Ktl::Vector<3> maxLim = {MAX_LIMIT_X, MAX_LIMIT_Y, MAX_LIMIT_Z};

Stage::Stage() : mode(MODE_IDLE), state(HOME_POS_STATE_IDLE), timeAtAlarmClearing(0.0), countClear(false){
  //Ktl::Vector<3> offset = Ktl::Vector<3>(65.0, 0.0, 250.49);
  Ktl::Vector<3> offset = Ktl::Vector<3>(82.0+350, 0.0, 243.0);
  
  //原点から見たヨー軸取り付け位置
  set(0.0, Ktl::X, Ktl::X, 1);
  set(0.0, Ktl::Y, Ktl::Y, 1);
  set(0.0, Ktl::Z, Ktl::Z, 1, offset);

  init();
  forward_kinematics();
}

/**
 * @brief アラーム状態設定処理
 *
 * @param motor モーター
 * @param alarmState アラーム状態
 */
void Stage::SetAlarmState(Ktl::Direction motor, bool alarmState){
  alarm[motor] = alarmState;
}

bool Stage::GetAlarmState(Ktl::Direction motor){
  return alarm[motor];
}

/**
 * @brief リミットセンサー状態設定処理
 *
 * @param axis 軸
 * @param alarmState リミットセンサー状態
 */
void Stage::SetLimitSensorState(Ktl::Direction axis, bool limitState){
  limitSensor[axis] = limitState;
}

/**
 * @brief 制御速度取得処理
 *
 * @param motor モーター
 * @return double 指定モーターの制御速度
 */
double Stage::GetControlSpeed(Ktl::Direction motor){
  return fabs(Pdref[motor]) * STAGE_SPEED_COEF;
}

/**
 * @brief 制御方向取得処理
 *
 * @param motor モーター
 * @return bool 指定モーターの制御方向
 */
bool Stage::GetControlDirection(Ktl::Direction motor){
  if( motor != Ktl::Direction::Z )
    return (Pdref[motor] <= 0);
  else  // Zはステージの回転方向と座標が逆
    return (Pdref[motor] > 0);
}

/**
 * @brief アラーム解除信号取得
 *
 * @return double アラーム解除信号の値
 */
double Stage::GetAlarmClearSignal(){
  if( flagAlarmClear )
    return ALARM_SIGNAL_VALUE_ON;
  else
    return ALARM_SIGNAL_VALUE_OFF;
}

/**
 * @brief ステージ位置取得
 *
 * @return Ktl::Vector<3> Pdref
 */
Ktl::Vector<3> Stage::GetPdref() const{
  return Pdref;
}

OperationMode Stage::GetOperationMode() const{
  OperationMode opMode = OP_MODE_IDLE;

  switch( mode ){
  case MODE_IDLE:
  case MODE_FORCED_STOP:
    // opMode = OP_MODE_IDLE;
    break;
  case MODE_CARTESIAN_MOTION:
  case MODE_VIEW_MOTION:
  case MODE_MANUAL_CONTROL:
    opMode = OP_MODE_MANUAL_CONTROL;
    break;
  case MODE_HOME_POS:
  case MODE_AUTO_HOME_POS:
    opMode = OP_MODE_HOME_POSITIONING;
    break;
  case MODE_VISUAL_SERVO:
    opMode = OP_MODE_VISUAL_SERVO;
    break;
  case MODE_SAFETY_CONTROL:
    opMode = OP_MODE_SAFETY_CONTROL;
    break;
  default:
    break;
  }

  return opMode;
}

/**
 * @brief 移動制御。リニアに加速する
 *
 * @param opMode 制御モード
 * @param opDir 制御軸方向
 * @param direction 制御方向
 * @param speed 速度
 * @param time タイムスタンプ
 */
void Stage::Move(MotionMode motion, int opDir, int direction, double speed, double time){
  if( mode != MODE_SAFETY_CONTROL ){
    switch( motion ){
    case MOTION_CARTESIAN:
      mode = MODE_CARTESIAN_MOTION;
      nextSpeed.zero();
      nextSpeed[opDir] = speed;
      speedOperator.turnUp(time, direction);
      break;
    case MOTION_VIEW:
      mode = MODE_VIEW_MOTION;
      nextSpeed.zero();
      nextSpeed[opDir] = speed;
      speedOperator.turnUp(time, direction);
      break;
    default:
      // Unexpected operation mode
      break;
    }
  }
  else{
    // Safety controlを継続する
  }
}

/**
 * @brief 停止
 *
 * @param time タイムスタンプ
 */
void Stage::Stop(double time){
  speedOperator.turnDown(time);
}

/**
 * @brief アラーム解除
 *
 * @param time アラーム解除指示時タイムスタンプ
 */
void Stage::ClearAlarm(double time){
  flagAlarmClear = true;
  timeAtAlarmClearing = time;
}

/**
 * @brief ホームポジショニング開始
 */
void Stage::StartHomePositioning(){
  if( mode != MODE_SAFETY_CONTROL ){
    mode = MODE_HOME_POS;
  }
  else{
    // Safety controlを継続
  }
}

/**
 * @brief ビジュアルサーボ開始
 */
void Stage::StartVisualServo(){
  if( mode != MODE_SAFETY_CONTROL ){
    mode = MODE_VISUAL_SERVO;
  }
  else{
    // Safety controlを継続
  }
}

/**
 * @brief 手動制御
 */
void Stage::ManualControl(){
  if( mode != MODE_SAFETY_CONTROL ){
    mode = MODE_MANUAL_CONTROL;
  }
  else{
    // Safety controlを継続
  }
}

void Stage::StartSafetyControl(Controller* ctrl, double time){
  Pdref.zero();
  mode = MODE_SAFETY_CONTROL;
  state = HOME_POS_STATE_IDLE;
  nextSpeed.zero();
  nextSpeed[Z] = ctrl->escapeSpeed;
  speedOperator.turnUp(time, -1);
}

/**
 * @brief 手動制御停止
 */
void Stage::StopManualControl(){
  mode = MODE_IDLE;
}

/**
 * @brief 強制停止
 */
void Stage::ForcedStop(){
  mode = MODE_FORCED_STOP;
}

/**
 * @brief 制御パラメータ更新処理
 *
 * @param ctrl
 * @param time
 */
void Stage::UpdateControlParameter(Controller* ctrl, double time){  /// @todo 引数を再検討する
  countClear = false;

  if( flagAlarmClear &&
      ((time - timeAtAlarmClearing) > ALARM_CLEAR_SIGNAL_PERIOD) ){
    flagAlarmClear = false;
  }


  if( ctrl->flagJoystickMotion ){

    // if( !ctrl->safetyControlEnabled ||
    //(ctrl->safetyStatus <= SAFETY_STATUS_CAUTION) ){
    //Ktl::Vector<3> input = ctrl->footCtrl.GetValues();
    Pdref = GetViewMotionVector(ctrl->Pdv, ctrl);

    ctrl->imageRotation += ctrl->wroll*DEG* 0.001;//ctrl->dt;

  }
  else{
    Pdref.zero();
  }
  
  switch( mode ){
    case MODE_IDLE:
      if( ctrl->autoHomePosEnabled &&
          (!ctrl->autoHomePosOnlyOutside || (ctrl->insertionStatus == INSERTION_STATUS_OUTSIDE)) &&
          ctrl->footSwitch.isOn() &&
          !IsAtHome() ){
        mode = MODE_AUTO_HOME_POS;
      }
      break;

    case MODE_CARTESIAN_MOTION:
      if( !ctrl->safetyControlEnabled ||
          (ctrl->safetyStatus <= SAFETY_STATUS_CAUTION) ){
        if( speedOperator.isActivated() ){
          Pdref = nextSpeed * speedOperator.value(time);
          if( ctrl->safetySlowDownEnabled &&
              (ctrl->safetyStatus != SAFETY_STATUS_SAFE) )
            Pdref *= ctrl->safetySlowDownSpeedCoeff;
          if( ctrl->dropOffPreventingEnabled &&
              (ctrl->insertionStatus == INSERTION_STATUS_EDGE) )
            ConvertToDropOffPreventingSpeed(Pdref, ctrl->endoscope.pose.column(Z));
          if( IsMovingOverLimit(Pdref) )
            Pdref.zero();
        }
        else{
          Pdref.zero();
          mode = MODE_IDLE;
        }
      }
      else{
        StartSafetyControl(ctrl, time);
      }
      break;

    case MODE_VIEW_MOTION:
      if( !ctrl->safetyControlEnabled ||
          (ctrl->safetyStatus <= SAFETY_STATUS_CAUTION) ){
        if( speedOperator.isActivated() ){
          Ktl::Vector<3> Pdv = nextSpeed * speedOperator.value(time);
          Pdref = GetViewMotionVector(Pdv, ctrl);

          if( ctrl->safetySlowDownEnabled &&
              (ctrl->safetyStatus != SAFETY_STATUS_SAFE) )
            Pdref *= ctrl->safetySlowDownSpeedCoeff;
          if( ctrl->dropOffPreventingEnabled &&
              (ctrl->insertionStatus == INSERTION_STATUS_EDGE) )
            ConvertToDropOffPreventingSpeed(Pdref, ctrl->endoscope.pose.column(Z));
          if( IsMovingOverLimit(Pdref) )
            Pdref.zero();
        }
        else{
          Pdref.zero();
          mode = MODE_IDLE;
        }
      }
      else{
        StartSafetyControl(ctrl, time);
      }
      break;

    case MODE_HOME_POS:  /// @todo ロジックを見直すこと
      if( !ctrl->safetyControlEnabled ||
          (ctrl->safetyStatus == SAFETY_STATUS_SAFE) ){
        switch( state ){
          case HOME_POS_STATE_IDLE:
            if( alarm[X] | alarm[Y] | alarm[Z] ){  // いずれかのアラームがセットされている
              state = HOME_POS_CLEAR_ALARM;
              if( !flagAlarmClear ){  // アラーム解除中でない
                ClearAlarm(time);
              }
            }
            else{
                state = HOME_POS_STATE_MOVING_TO_STOP_POS;
            }
            break;
          case HOME_POS_CLEAR_ALARM:
            if( !flagAlarmClear )
              state = HOME_POS_STATE_MOVING_TO_STOP_POS;
            break;
          case HOME_POS_STATE_MOVING_TO_STOP_POS:
            if( (limitSensor[X] & limitSensor[Y] & limitSensor[Z]) == false ){
              Pdref = Ktl::Vector<3>(-ctrl->homePosSpeed * (int)(!limitSensor[X]),
                                     -ctrl->homePosSpeed * (int)(!limitSensor[Y]),
                                     -ctrl->homePosSpeed * (int)(!limitSensor[Z]));
            }
            else{
              countClear = true;
              Pdref.zero();
              state = HOME_POS_STATE_MOVING_TO_HOME_POS;
            }
            break;
          case HOME_POS_STATE_MOVING_TO_HOME_POS:{
            if( ctrl->safetyStatus == SAFETY_STATUS_SAFE ){
              Ktl::Vector<3> posDiff = homePos - getq();
              double d = 0;

              for( int i = 0; i < 3; i++ ){
                if (posDiff[i] > 0.5)
                    Pdref[i] = ctrl->homePosSpeed;
                else if(posDiff[i] < -0.5)
                    Pdref[i] = -ctrl->homePosSpeed;
                else
                    Pdref[i] = (posDiff[i]) * ctrl->homePosSpeed;
                d += fabs(posDiff[i]);
              }
              if (d < 0.00001 && d > -0.00001){
                countClear = true;
                Pdref.zero();
                state = HOME_POS_STATE_IDLE;
                mode = MODE_IDLE;
              }
              break;
            }
            else{
              Pdref.zero();
              mode = MODE_IDLE;
            }
          }
          default:
            break;
        }
      }
      else{
        StartSafetyControl(ctrl, time);
      }
      break;

    case MODE_AUTO_HOME_POS:{
      if( (!ctrl->autoHomePosOnlyOutside || (ctrl->insertionStatus == INSERTION_STATUS_OUTSIDE)) &&
          ctrl->footSwitch.isOn() ){
        Ktl::Vector<3> posDiff = getq();
        for( int i = 0; i < 3; i++ ){
          if( posDiff[i] > 5 )
            Pdref[i] = -ctrl->autoHomePosSpeed;
          else if( posDiff[i] < -5 )
            Pdref[i] = ctrl->autoHomePosSpeed;
          else
            Pdref[i] = 0.0;
        }
        if( Pdref.abs2() == 0.0 ){
          mode = MODE_IDLE;
        }
      }
      else{
        Pdref.zero();
        mode = MODE_IDLE;
      }
      break;
    }

    case MODE_VISUAL_SERVO:{
      if( !(ctrl->footSwitch.isOn()) ){
        if( !ctrl->safetyControlEnabled ||
            (ctrl->safetyStatus == SAFETY_STATUS_SAFE) ){
          Ktl::Vector<3> forcepsPos = ctrl->forceps.GetPosition();
          if( forcepsPos.abs() > VISUAL_SERVO_DEADBAND_THRESH ){
            forcepsPos.rotate(Z, ctrl->imageRotation * PI / 180);
            Pdref = GetViewMotionVector(forcepsPos * ctrl->visualServoSpeed, ctrl);
            // if( ctrl->dropOffPreventingEnabled &&
            //     (ctrl->insertionStatus == INSERTION_STATUS_EDGE) )
            //   Pdref = Pdref + ctrl->endoscope.pose.column(Z) * ctrl->escapeSpeed;  // 要検討
            if( IsMovingOverLimit(Pdref) )
              Pdref.zero();
          }
          else{
            Pdref.zero();
          }
        }
        else if( ctrl->safetyStatus == SAFETY_STATUS_CAUTION ){
          Pdref.zero();
          mode = MODE_IDLE;
        }
        else{
          StartSafetyControl(ctrl, time);
        }
      }
      else{
        Pdref.zero();
        mode = MODE_IDLE;
      }
      break;
    }

    case MODE_MANUAL_CONTROL:{
      if( !ctrl->safetyControlEnabled ||
          (ctrl->safetyStatus <= SAFETY_STATUS_CAUTION) ){
        Ktl::Vector<3> input = ctrl->footCtrl.GetValues();
        // input.print();
        if( input.abs() > MANUAL_DEADBAND_THRESH ){
          input = input - (input * MANUAL_DEADBAND_THRESH);
          Ktl::Vector<3> Pdv = Ktl::Vector<3>(input[VIEW_MOTION_DIRECTION_YAW] * ctrl->footControlSpeed[VIEW_MOTION_DIRECTION_YAW],
                                              input[VIEW_MOTION_DIRECTION_PITCH] * ctrl->footControlSpeed[VIEW_MOTION_DIRECTION_PITCH],
                                              input[VIEW_MOTION_DIRECTION_ZOOM] * ctrl->footControlSpeed[VIEW_MOTION_DIRECTION_ZOOM]);
          Pdref = GetViewMotionVector(Pdv, ctrl);
          if( ctrl->safetySlowDownEnabled &&
              (ctrl->safetyStatus != SAFETY_STATUS_SAFE) )
            Pdref *= ctrl->safetySlowDownSpeedCoeff;
          if( ctrl->dropOffPreventingEnabled &&
             (ctrl->insertionStatus == INSERTION_STATUS_EDGE) )
            ConvertToDropOffPreventingSpeed(Pdref, ctrl->endoscope.pose.column(Z));
          if( IsMovingOverLimit(Pdref) )
            Pdref.zero();
        }
        else{
          Pdref.zero();
          mode = MODE_IDLE;
        }
      }
      else{
        StartSafetyControl(ctrl, time);
      }
      break;
    }

    case MODE_SAFETY_CONTROL:
      if( (ctrl->safetyStatus != SAFETY_STATUS_SAFE) &&
          !(ctrl->footSwitch.isOn()) ){
        if( speedOperator.isActivated() ){
          Ktl::Vector<3> Pdv = nextSpeed * speedOperator.value(time);
          Pdref = ctrl->endoscope.pose.column(Z) * Pdv[Z];
        }
        if( IsMovingOverLimit(Pdref) )
          Pdref.zero();
        else{
          Pdref.zero();
          mode = MODE_IDLE;
        }
      }
      else{
        Pdref.zero();
        mode = MODE_IDLE;
      }
      break;

    case MODE_FORCED_STOP:
      Pdref.zero();
      mode = MODE_IDLE;
      state = HOME_POS_STATE_IDLE;
      break;

    default:
      break;
  }//switch




  
}

/**
 * @brief カウントクリアフラグ取得
 *
 * @return true カウント値をクリアする
 * @return false カウント値をクリアしない
 */
bool Stage::CountClear(){
  return countClear;
}

bool Stage::IsAtLimit(Ktl::Direction axis, double position) const{
  if( (position < minLim[axis]) ||
      (position > maxLim[axis]) ){
    return true;
  }
  else
    return false;
}

Ktl::Vector<3> Stage::GetViewMotionVector(Ktl::Vector<3> Pdv, Controller* ctrl)
{
  Pdv[X] *= -1;
  Ktl::Vector<3> Pp = ctrl->endoscope.pivotPos;

  Ktl::Vector<3> ws( 0.02*Pdv[Y], -0.02*Pdv[X], 0.0);
  Ktl::Vector<3> r =  ctrl->Pr - Pp; //外側の旋回半径ベクトル
  Ktl::Vector<3> wref = ctrl->endoscope.pose * ws; //連続運動学

  Ktl::Vector<3> Pdref0 = wref * r
    + Pdv[Z] * ctrl->endoscope.pose.column(Z);

  Ktl::Matrix<3,3> Jr3;
  Ktl::Matrix<3,ADOF> Jr = ctrl->arm.Jr();
  for(int j=0;j<3;j++) //ジンバルによる姿勢ヤコビ行列
    Jr3.column(j, Jr.column(j+3) );

  Ktl::Vector<3> qdref3;
  if( !qdref3.solve( Jr3, wref ) ){//- joint[0].qdref*armFK.s[0])){
    printf("decompose2 \n");//%d %d \n", ret, count);
  }
  double drollref = qdref3[2]; //ロール回転速度

  ctrl->imageRotation += drollref*DEG* 0.001;//ctrl->dt;
  //ロール補正
  //xprintf("roll %f %f\n",drollref,ctrl->imageRotation);

  //----------------------------------------------
  double theta = asin( 0.5*Pdref0.abs() / r.abs() );

  Ktl::Vector<3> Pdref1 =
    Ktl::Matrix<3,3>( Ktl::normal(-Pdref0 * r ), theta ) * Pdref0;
  Ktl::Vector<3> n = ctrl->endoscope.pose.column(2);

  double d = r.abs() - ctrl->r0;
  if( abs(Pdref0 & n) < 1e-10 )  // 視野の平行移動の場合
    Pdref = Pdref0 + 0.5 * Pdref0.abs() * d * n;//軸方向の移動量を補正
  else                           // 挿入方向の移動を含む場合
    Pdref = Pdref0;

  return Pdref;
}

void Stage::ConvertToDropOffPreventingSpeed(Ktl::Vector<3> &speed, Ktl::Vector<3> endoscopePose){
  if( (speed & endoscopePose) < 0 )  // 抜去方向に動作する場合
    speed -= (speed | endoscopePose);
}

bool Stage::IsAtHome(){
  Ktl::Vector<3> posDiff = homePos - getq();
  for( int i = 0; i < 3; i++ ){
    if( abs(posDiff[i]) > 10.0 )
      return false;
  }
  return true;
}

bool Stage::IsMovingOverLimit(Ktl::Vector<3> speed){
  Ktl::Vector<3> position = getq();

  for( int i = 0; i < 3; i++ ){
    if( ((position[i] < minLim[i]) && (speed[i] < 0.0)) ||
        ((position[i] > maxLim[i]) && (speed[i] > 0.0)) ){
      return true;
    }
  }
  return false;
}
