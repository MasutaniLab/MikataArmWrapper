// -*- C++ -*-
/*!
 * @file  MikataArmWrapper.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#define _USE_MATH_DEFINES
#include <iostream>
#include <iomanip>
#include <cmath>
#include "MikataArmWrapper.h"
using namespace std;


//static double fix_the_error2 = 0.001535889741755; // 変換定数 サーボの値の1に対応するrad単位の角度(全サーボ共通)
//static double Fix_The_Error = 13.9;


// 定数
//const double addLength = 0.035;   // ID5から指先までの長さ
//const double addLength = 0.13;    // FV装着時の長さ(先端)
const double addLength = 0.10;      // FV装着時の長さ(面)
const double L1 = 0.0765;              // 角サーボ間の長さ[m]
const double L2 = 0.14993;             // [m]
const double L3 = 0.15;                // [m]
const double L4 = 0.04225 + addLength; // [m]
const double fix_ID1 = -M_PI;
const double fix_ID2 = -1.729411849216112;
const double fix_ID3 = -2.988841437455199;
const double fix_ID4 = -M_PI;
const double fix_ID5 = M_PI;
//static double pos2rad[5] = { fix_ID1, fix_ID2, fix_ID3, fix_ID4, fix_ID5 }; // 各サーボの内部値が0の時の理論的な角度, 単位[rad]

const double pos2rad[5] = { (2*M_PI)/4096, (2 * M_PI) / 4096, (2 * M_PI) / 4096, (2 * M_PI) / 4096, (2 * M_PI) / 4096 };

const double posoffset[5] = { 2048, 1024, 2048, 2048, 2048 };               // 運動学のモデル(内部表現がすべて0なら 地面と水平な状態)                                                                         
const double rps2speed = 1024 / 234.4*60.0 / (2 * M_PI);              //rad/s -> moving speed


//const double handClosed = 0.5;  //[rad] <-閉じすぎて危険
//const double handOpen = -0.75;   //[rad]

//const double handClosed = 0.003;  //[rad] 2048くらい

const double handClosed = 0.037;  //[rad] これ以上強く握るとerror
const double handOpen = -0.9;

const int StatusBitMoving = 1;
const int StatusBitExecuting = 2;
const int StatusBitCommandError = 4;
const int StatusBitLowerError = 8;

//プロトタイプ宣言
void directKinematics(double &x, double &y, double &z, double &pitch, double q[4]);
bool inverseKinematics(double q[4], double x, double y, double z, double pitch, const int mode);
double normalizeAngle(double x);


// Module specification
// <rtc-template block="module_spec">
static const char* mikataarmwrapper_spec[] =
  {
    "implementation_id", "MikataArmWrapper",
    "type_name",         "MikataArmWrapper",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "VenderName",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
MikataArmWrapper::MikataArmWrapper(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_armTipTargetIn("armTipTarget", m_armTipTarget),
    m_armJointTargetIn("armJointTarget", m_armJointTarget),
    m_presentPositionIn("presentPosition", m_presentPosition),
    m_movingIn("moving", m_moving),
    m_armTipOut("armTip", m_armTip),
    m_armJointOut("armJoint", m_armJoint),
    m_goalPositionOut("goalPosition", m_goalPosition),
    m_movingSpeedOut("movingSpeed", m_movingSpeed),
    m_armStatusOut("armStatus", m_armStatus)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
MikataArmWrapper::~MikataArmWrapper()
{
}



RTC::ReturnCode_t MikataArmWrapper::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("armTipTarget", m_armTipTargetIn);
  addInPort("armJointTarget", m_armJointTargetIn);
  addInPort("presentPosition", m_presentPositionIn);
  addInPort("moving", m_movingIn);
  
  // Set OutPort buffer
  addOutPort("armTip", m_armTipOut);
  addOutPort("armJoint", m_armJointOut);
  addOutPort("goalPosition", m_goalPositionOut);
  addOutPort("movingSpeed", m_movingSpeedOut);
  addOutPort("armStatus", m_armStatusOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MikataArmWrapper::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MikataArmWrapper::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MikataArmWrapper::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t MikataArmWrapper::onActivated(RTC::UniqueId ec_id)
{
    cout << "MikataArmWrapper::onActivated()" << endl;

    m_armTip.data.length(5);
    m_armJoint.data.length(m_NUM_ACTUATOR);
    m_goalPosition.data.length(m_NUM_ACTUATOR);
    m_movingSpeed.data.length(m_NUM_ACTUATOR);

    //初期の目標
    for (int i = 0; i<m_NUM_ACTUATOR; i++) { m_jointTarget[i] = 0; }
    m_jointTarget[1] = M_PI / 2; //直立状態
    m_jointTarget[4] = handOpen;
    m_time = 1.0;
    m_newTarget = false;
    m_movingPrev = false;
    m_errorPrev = false;
    m_rangeError = false;
    m_stopCount = 0;
    m_state = Start;
    m_statePrev = Stop;
    m_timeStart = coil::gettimeofday();
    m_ikError = false;
    m_newCommand = false;
    m_init = true;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t MikataArmWrapper::onDeactivated(RTC::UniqueId ec_id)
{
    cout << "MikataArmWrapper::onDeactivated()" << endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MikataArmWrapper::onExecute(RTC::UniqueId ec_id)
{
    //関節指令
    if (m_armJointTargetIn.isNew()) {
        cout << "m_armJointTargetIn.isNew()" << endl;
        m_armJointTargetIn.read();
        for (int i = 0; i<m_NUM_ACTUATOR; i++){
            m_jointTarget[i] = m_armJointTarget.data[i];
            cout << i << " " << m_jointTarget[i] << endl;
        }
        m_time = m_armJointTarget.data[m_NUM_ACTUATOR];
        cout << "mitme: " << m_time << endl;
        if (m_time < 0) { m_time = 0; }
        m_newTarget = true;
    }

    //手先指令
    if (m_armTipTargetIn.isNew()) {
        m_armTipTargetIn.read();
        cout << "m_armTipTargetIn.isNew()" << endl;
        double x = m_armTipTarget.data[0];
        double y = m_armTipTarget.data[1];
        double z = m_armTipTarget.data[2];
        double pitch = m_armTipTarget.data[3];
        double hand = m_armTipTarget.data[4];
        m_time = m_armTipTarget.data[5];

        cout << "x: " << x << endl;
        cout << "y: " << y << endl;
        cout << "z: " << z << endl;
        cout << "pitch: " << pitch << endl;
        cout << "hand: " << hand << endl;
        cout << "m_time: " << m_time << endl;

        if (m_time < 0) { m_time = 0; }

        //逆運動学を解き，関節の目標を求める
        m_ikError = false;
        if (pitch < -M_PI || pitch > M_PI)
        {
            m_ikError = true;
        }
        else {
            int mode = 1;
#if 0
            int mode = 0;

            if (z < 0.125) { //z座標が下方であれば肘を上にする（関節2を負に）
                mode = 1;
            }
#endif
            m_ikError = inverseKinematics(m_jointTarget, x, y, z, pitch, mode);
        }
        cout << "m_ikError: " << m_ikError << endl;

        //現在角度を考慮して関節速度の上限を決める
        if (!m_ikError) {
            if (hand < 0) {
                hand = 0;
            }
            else if (hand > 1) {
                hand = 1;
            }
            m_jointTarget[4] = hand*(handOpen - handClosed) + handClosed;
            cout << "m_jointTarget:" << endl;
            for (int i = 0; i<m_NUM_ACTUATOR; i++) { cout << i << ": " << m_jointTarget[i] << endl; }
            m_newTarget = true;
        }
    }

    //Dynamixelの指令に変換
    if (m_newTarget) { //新しい指令値があれば
        m_rangeError = false;
        for (int i = 0; i<m_NUM_ACTUATOR; i++) {
            //角度を内部表現に変換
            int p = int(m_jointTarget[i] / pos2rad[i] + posoffset[i] + 0.5);
            if (p < 0 || 4095 < p) {
                m_rangeError = true;
            }
            m_goalPosition.data[i] = p;
            //速度を計算
			if (m_time == 0) {
				m_movingSpeed.data[i] = 0; //0の場合は最高速度
			}
			else {
				double dq;
				dq = (m_jointTarget[i] - m_jointCurrent[i]) / m_time; //[rad/s]
				cout << i << " dq: " << dq << " " << m_jointTarget[i] << " " << m_jointCurrent[i] << endl;
				//速度を内部表現に変換
				int s = abs(dq*rps2speed);
				if (s < 1) {
					s = 1;
				}
				else if (s > 2047) {
					s = 2047;
				}
				m_movingSpeed.data[i] = s;
			}
        }
        cout << "m_goalPosition.data:" << endl;
        for (int i = 0; i<m_NUM_ACTUATOR; i++) {
            cout << i << ": " << m_goalPosition.data[i] << endl;
        }
        cout << "m_movingSpeed.dat:" << endl;
        for (int i = 0; i<m_NUM_ACTUATOR; i++) {
            cout << i << ": " << m_movingSpeed.data[i] << endl;
        }
        //出力ポートに書き込み
        if (m_rangeError) {
            cout << "m_rangeError" << endl;
        }
        else {
            m_goalPositionOut.write();
            m_movingSpeedOut.write();
            m_newCommand = true;
        }
        m_newTarget = false;
    }

    //Dynamixelからの入力に対応
    if (m_presentPositionIn.isNew()) {
        m_presentPositionIn.read();
        //角度を内部表現からradに
        for (int i = 0; i<m_NUM_ACTUATOR; i++) {
            //Dynamixelからのデータが不良な場合を想定
            int p = m_presentPosition.data[i];
            if (p < 0 || 4095 < p) {
                cout << "m_presentPosition.data[" << i << "] : " << p << " 範囲外！" << endl;
                p = 2048;
            }
            m_jointCurrent[i] = (p - posoffset[i])*pos2rad[i];
            m_armJoint.data[i] = m_jointCurrent[i];
        }
        //出力ポートに書き込み
        m_armJointOut.write();
        //順運動学計算
        double x, y, z, pitch;
        directKinematics(x, y, z, pitch, m_jointCurrent);
        m_armTip.data[0] = x;
        m_armTip.data[1] = y;
        m_armTip.data[2] = z;
        m_armTip.data[3] = pitch;
        m_armTip.data[4] = (m_jointCurrent[4] - handClosed) / (handOpen - handClosed);
        //出力ポートに書き込み
        m_armTipOut.write();
    }

    if (m_movingIn.isNew()) {
        m_movingIn.read();
        bool moving = false;
        bool error = false;
        for (int i = 0; i<m_NUM_ACTUATOR; i++) {
            //cout << m_moving.data[i] << ",";
            if (m_moving.data[i] == 1) {
                moving = true;
            }
            else if (m_moving.data[i] == 3) {
                error = true;
            }
        }
        //cout << endl;
        if (moving != m_movingPrev) {
            if (moving) {
                cout << "動作中" << endl;
            }
            else {
                cout << "停止中" << endl;
            }
        }
        m_movingPrev = moving;
        if (error != m_errorPrev) {
            if (error) {
                cout << "下位でエラー発生" << endl;
            }
            else {
                cout << "下位でエラーなし" << endl;
            }
        }
        m_errorPrev = error;

        //状態遷移
        bool stateChanged = false;
        if (m_state != m_statePrev) {
            stateChanged = true;
        }
        m_statePrev = m_state;
        if (m_ikError || m_rangeError) {
            m_state = CommandError;
            cout << "--> CommandError" << endl;
        }
        if (error) {
            m_state = LowerError;
            cout << "--> LowerError" << endl;
        }
        //cout << "m_state: " << m_state << endl;
        if (m_state == Start) {
            if (stateChanged) {
                m_timeStart = coil::gettimeofday();
                m_newCommand = false;
            }
            if (moving) {//動き出したら
                m_state = Execute;
                cout << "Start --> Execute" << endl;
            }
            else {
                double elapsedTime = coil::gettimeofday() - m_timeStart;
                //cout << "elapsedTime: " << elapsedTime << endl;
                if (elapsedTime > 1.0) {//開始から一定期間止まっていたら
                    m_state = Stop;
                    cout << "Start --> Stop" << endl;
                }
            }
        }
        else if (m_state == Execute) {
            if (!moving) {//止まったら
                m_state = Stop;
                cout << "Execute  --> Stop" << endl;
            }
        }
        else if (m_state == Stop) {
            if (m_newCommand) {//新しい指令を受けたら
                m_state = Start;
                cout << "Stop  --> Start" << endl;
            }
        }
        else if (m_state == CommandError) {
            if (m_newCommand) {//新しい指令を受けたら
                m_state = Start;
                cout << "CommandError  --> Start" << endl;
            }
        }
        else if (m_state == LowerError) {
            if (!error && !moving) {//下位のエラーがなくなり止まっていたら
                m_state = Stop;
                cout << "LowerError  --> Stop" << endl;
            }
        }

        //armStatusポートの出力の決定
        m_armStatus.data = 0;
        if (moving) {
            m_armStatus.data |= StatusBitMoving;
        }
        if (m_state == Start || m_state == Execute) {
            m_armStatus.data |= StatusBitExecuting;
        }
        else if (m_state == CommandError) {
            m_armStatus.data |= StatusBitCommandError;
        }
        else if (m_state == LowerError) {
            m_armStatus.data |= StatusBitLowerError;
        }
        //cout << "m_armStatus.data: " << m_armStatus.data << endl;
        m_armStatusOut.write();
    }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MikataArmWrapper::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t MikataArmWrapper::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MikataArmWrapper::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MikataArmWrapper::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MikataArmWrapper::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


// 4回転関節アームの順運動学
// x,y,z,pitch  手先の位置と方向．【出力】
// q[4]  q[0]～q[3]が第1～4関節の角度．逆運動学の解．【入力】
// 戻り値 なし
void
directKinematics(double &x, double &y, double &z, double &pitch, double q[4])
{
    // 元
    //double q1 = q[0];
    //double q2 = q[0];
    //double q3 = q[0];
    //double q4 = q[0];

    // 新
    double q1 = q[0];
    double q2 = q[1];
    double q3 = q[2];
    double q4 = q[3];

    double c1 = cos(q1);
    double s1 = sin(q1);
    double c2 = cos(q2);
    double s2 = sin(q2);
    double c23 = cos(q2 + q3);
    double s23 = sin(q2 + q3);
    double c234 = cos(q2 + q3 + q4);
    double s234 = sin(q2 + q3 + q4);
    x = c1*(L2*c2 + +L3*c23 + L4*c234);
    y = s1*(L2*c2 + +L3*c23 + L4*c234);
    z = L1 + L2*s2 + +L3*s23 + L4*s234;
    pitch = q2 + q3 + q4;
}

// 4回転関節アームの逆運動学
// q[4]  q[0]～q[3]が第1～4関節の角度．逆運動学の解．【出力】
// x,y,z,pitch  手先の位置と方向．【入力】
// mode   逆運動学の解の種類．0: 第3関節が正，1: 第3関節が負．
// 戻り値 逆運動学が解ける場合はfalse，解けない場合はtrue．
bool
inverseKinematics(double q[4], double x, double y, double z, double pitch, const int mode)
{
    double q1, q2, q3, q4;
    q1 = atan2(y, x);
    double r = sqrt(x*x + y*y);
    double rr = r - L4*cos(pitch);
    double zz = z - L4*sin(pitch) - L1;
    double d = sqrt(rr*rr + zz*zz);
    double cq3 = (d*d - L2*L2 - L3*L3) / (2 * L2*L3);
    if (abs(cq3) > 1.0) return true;
    double phi = atan2(zz, rr);
    double psi = acos((d*d + L2*L2 - L3*L3) / (2 * L2*d));
    if (mode == 0) {
        q2 = normalizeAngle(phi - psi);
        q3 = acos(cq3);
    }
    else {
        q2 = normalizeAngle(phi + psi);
        q3 = -acos(cq3);
    }
    q4 = normalizeAngle(pitch - q2 - q3);
    q[0] = q1;
    q[1] = q2;
    q[2] = q3;
    q[3] = q4;

    return false;
}

// 引数の角度を-π～+πの間に正規化したものを返す
double
normalizeAngle(double x)
{
    if (x<-M_PI) x += M_PI * 2 * (double)((int)(-x / (2 * M_PI) + 0.5));
    else if (x> M_PI) x -= M_PI * 2 * (double)((int)(x / (2 * M_PI) + 0.5));
    return x;
}

extern "C"
{
 
  void MikataArmWrapperInit(RTC::Manager* manager)
  {
    coil::Properties profile(mikataarmwrapper_spec);
    manager->registerFactory(profile,
                             RTC::Create<MikataArmWrapper>,
                             RTC::Delete<MikataArmWrapper>);
  }
  
};


