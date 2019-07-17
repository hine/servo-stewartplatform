/**
 * file sp_robot.cpp
 * Library for GdW's Stewart-platform Robot
 * @author Daisuke IMAI <hine.gdw@gmail.com>
 * @date 2019/07/15
 * @version 1.0.0
 * @url https://github.com/hine/servo-stewartplatform
 */
#include <Arduino.h>
#include <vs-rc202.h>
#include <EEPROM.h>
#include "sp_robot.h"


// サーボの関連設定(VS-RC202用)
const float SERVO_ANGLE_LIMIT_MIN = -90.0;
const float SERVO_ANGLE_LIMIT_MAX = 90.0;
const float SERVO_ANGLE_RATIO = 19.0;


/**
 * StewartPlatformクラスのコンストラクタ
 */
StewartPlatform::StewartPlatform() {
  // 初期化済みフラグの初期化(未初期化状態)
  is_initialized_ = false;
}

// StewartPlatformクラスの定数定義

// 角度計算を少し楽にするための定数
const float StewartPlatform::RAD2DEG = 180.0 / PI;
const float StewartPlatform::DEG30 = PI / 6.0;

// スチュワートプラットフォームでのサーボの上限下限値
const float StewartPlatform::SERVO_ROTATION_MIN = radians(-40.0);
const float StewartPlatform::SERVO_ROTATION_MAX = radians(90.0);

// ベース(下面側)：サーボとサーボアームの取り付け位置に関する定数
// サーボアームの指す方向(x軸に対して)(sp_beta)
const float StewartPlatform::SERVO_ARM_DIRECTIONS[] = {11 * PI / 18, -11 * PI / 18, -PI / 18, 13 * PI / 18, -13 * PI / 18, PI / 18};
// サーボ軸間の角度(sp_theta_p)
const float StewartPlatform::ANGLE_BETWEEN_SERVOS = radians(52.0);
// 計算を楽にするために保持する変数(sp_theta_angle)
const float StewartPlatform::ANGLE_GAP_TO_SERVO = (PI / 3 - ANGLE_BETWEEN_SERVOS) / 2;
// サーボアームの長さ(mm)(SP_L1)
const float StewartPlatform::SERVO_ARM_LENGTH = 30.0;
// ベースの中心からアームとリンクの接続位置までの距離(mm)(SP_PD)
const float StewartPlatform::BASE_CENTER_TO_LINK_RADIUS = 60.0;

// 上記の設定を用いて事前にベース側のリンク接続点の標準位置を計算(z位置はゼロのため持たない)(sp_p)
const float StewartPlatform::BASE_LINK_POINTS[6][2] = {
  {
    BASE_CENTER_TO_LINK_RADIUS * cos(DEG30 + ANGLE_GAP_TO_SERVO),
    BASE_CENTER_TO_LINK_RADIUS * sin(DEG30 + ANGLE_GAP_TO_SERVO)
  },
  {
    BASE_CENTER_TO_LINK_RADIUS * cos(DEG30 + ANGLE_GAP_TO_SERVO),
    -BASE_CENTER_TO_LINK_RADIUS * sin(DEG30 + ANGLE_GAP_TO_SERVO)
  },
  {
    BASE_CENTER_TO_LINK_RADIUS * sin(ANGLE_GAP_TO_SERVO),
    -BASE_CENTER_TO_LINK_RADIUS * cos(ANGLE_GAP_TO_SERVO)
  },
  {
    -BASE_CENTER_TO_LINK_RADIUS * cos(DEG30 - ANGLE_GAP_TO_SERVO),
    -BASE_CENTER_TO_LINK_RADIUS * sin(DEG30 - ANGLE_GAP_TO_SERVO)
  },
  {
    -BASE_CENTER_TO_LINK_RADIUS * cos(DEG30 - ANGLE_GAP_TO_SERVO),
    BASE_CENTER_TO_LINK_RADIUS * sin(DEG30 - ANGLE_GAP_TO_SERVO)
  },
  {
    BASE_CENTER_TO_LINK_RADIUS * sin(ANGLE_GAP_TO_SERVO),
    BASE_CENTER_TO_LINK_RADIUS * cos(ANGLE_GAP_TO_SERVO)
  }
};

// プラットフォーム(上面側)：リンク接続位置に関する定数
// プラットフォームの中心からリンクの接続位置までの距離(mm)(SP_RD)
const float StewartPlatform::PLATFORM_CENTER_TO_LINK_RADIUS = 62.0;
// プラットフォームでの接続点間の角度(sp_theta_r)
const float StewartPlatform::ANGLE_BETWEEN_LINK_CONNECTIONS = radians(11.0);

// 上記の設定を用いて事前にプラットフォーム側のリンク接続点の標準位置を計算(z位置はプラットフォームの相対位置のためゼロ)(sp_re)
const float StewartPlatform::PLATFORM_LINK_POINTS[6][3] = {
  {
    PLATFORM_CENTER_TO_LINK_RADIUS * cos(ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    PLATFORM_CENTER_TO_LINK_RADIUS * sin(ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    0
  },
  {
    PLATFORM_CENTER_TO_LINK_RADIUS * cos(ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    -PLATFORM_CENTER_TO_LINK_RADIUS * sin(ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    0
  },
  {
    -PLATFORM_CENTER_TO_LINK_RADIUS * sin(DEG30 - ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    -PLATFORM_CENTER_TO_LINK_RADIUS * cos(DEG30 - ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    0
  },
  {
    -PLATFORM_CENTER_TO_LINK_RADIUS * sin(DEG30 + ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    -PLATFORM_CENTER_TO_LINK_RADIUS * cos(DEG30 + ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    0
  },
  {
    -PLATFORM_CENTER_TO_LINK_RADIUS * sin(DEG30 + ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    PLATFORM_CENTER_TO_LINK_RADIUS * cos(DEG30 + ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    0
  },
  {
    -PLATFORM_CENTER_TO_LINK_RADIUS * sin(DEG30 - ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    PLATFORM_CENTER_TO_LINK_RADIUS * cos(DEG30 - ANGLE_BETWEEN_LINK_CONNECTIONS / 2),
    0
  }
};

/**
 * サーボなどの初期化処理
 *  
 * @param fps[float] ロボットの処理のFPS 
 */
void StewartPlatform::init(float fps) {
  // VS-RC202の初期化
  initLib();
  setServoLimitL((int)(SERVO_ANGLE_LIMIT_MIN * SERVO_ANGLE_RATIO));
  setServoLimitH((int)(SERVO_ANGLE_LIMIT_MAX * SERVO_ANGLE_RATIO));

  // ブザーの有効化
  buzzerEnable(1);

  // EEPROMからデータの読み出し
  EEPROM.begin(20);
  EEPROM.get<RobotBodyConfig>(0, robot_body_config_);

  // EEPROMからのデータのヘッダを確認
  if (robot_body_config_.header1 != 's' || robot_body_config_.header2 != 'p') {
    // ヘッダにspがついていない場合は未設定と思われるのでブザーでエラーを知らせる
    while(true) {
      setBuzzer(PC4, BEAT2, TANG);
      delay(500);
    }
  }

  // EEPROMのリンク長からplatform_z_homeを計算する
  link_length_ = robot_body_config_.link_length;
  platform_z_home_ = calculatePlatformZHome_(link_length_);
  
  // VS-RC202関連設定
  fps_ = fps;
  setServoMovingTime(constrain(int(1000.0 / fps_), 20, 100000)); // サーボの目的位置への移動時間を設定
  setServoMode(0); // オーバーライドモードに（次の命令が来たら即時変更する）

  // サーボ初期化  
  sp_servo_[0].setParameters(1, -1, robot_body_config_.servo1_offset, SERVO_ROTATION_MIN, SERVO_ROTATION_MAX, fps_);
  sp_servo_[1].setParameters(2, 1, robot_body_config_.servo2_offset, SERVO_ROTATION_MIN, SERVO_ROTATION_MAX, fps_);
  sp_servo_[2].setParameters(3, -1, robot_body_config_.servo3_offset, SERVO_ROTATION_MIN, SERVO_ROTATION_MAX, fps_);
  sp_servo_[3].setParameters(4, 1, robot_body_config_.servo4_offset, SERVO_ROTATION_MIN, SERVO_ROTATION_MAX, fps_);
  sp_servo_[4].setParameters(5, -1, robot_body_config_.servo5_offset, SERVO_ROTATION_MIN, SERVO_ROTATION_MAX, fps_);
  sp_servo_[5].setParameters(6, 1, robot_body_config_.servo6_offset, SERVO_ROTATION_MIN, SERVO_ROTATION_MAX, fps_);


  for (uint8_t sid = 0; sid < 6; sid++) {
    // vsidごとにサーボのオフセットをセットする
    setServoOffset(sp_servo_[sid].getVsid(), sp_servo_[sid].getOffset());
    // vsidごとにサーボへのPWM出力を可能にする
    servoEnable(sp_servo_[sid].getVsid(), 1);
    // vsidからの逆引きテーブル作成
    vsid_table[sp_servo_[sid].getVsid()] = sid;
  }

  // 初期化済みフラグを立てる
  is_initialized_ = true;

  // 正常起動音
  setBuzzer(PB5, BEAT8, TANG);
  setBuzzer(PB6, BEAT8, TANG);
}

/**
 * 定期実行から呼び出されるロボット全体の状態変更
 */
void StewartPlatform::changeState() {
  if (is_initialized_) {
    // 次の瞬間のためのサーボ情報のための変数を用意
    int next_position[7];
    next_position[0] = int(1000 / fps_); // 
    for (uint8_t sid = 0; sid < 6; sid++) {
      sp_servo_[sid].changeState();
      next_position[sid + 1] = int(sp_servo_[sid].getCurrentAngle() * RAD2DEG * sp_servo_[sid].getReverse() * SERVO_ANGLE_RATIO);
    }
    setMotion(next_position, 6);
    moveServo();
  }
}

/**
 * サーボ単体で角度指示を送りサーボを動かす
 * 
 * @param vsid[uint8_t] サーボID(V-duinoのサーボ番号)
 * @param angle[float] 目標角度(角度) ※(ラジアン)でないことに注意
 * @param moving_time[uint16_t] 移動にかける時間(ms)
 */
void StewartPlatform::setServoAngle(uint8_t visd, float angle, uint16_t moving_time) {
  if (is_initialized_) {
    sp_servo_[vsid_table[visd]].setTargetAngle(radians(constrain(angle, SERVO_ANGLE_LIMIT_MIN, SERVO_ANGLE_LIMIT_MAX)), moving_time);
  }
}

/**
 * IKで位置と姿勢を指示する
 * 
 * @param pos_x[float] 目標位置のx軸位置(mm)
 * @param pos_y[float] 目標位置のy軸位置(mm)
 * @param pos_z[float] 目標位置のz軸位置(mm)
 * @param rot_x[float] 目標姿勢のx軸の傾き(角度) ※(ラジアン)でないことに注意
 * @param rot_y[float] 目標姿勢のy軸の傾き(角度) ※(ラジアン)でないことに注意
 * @param rot_z[float] 目標姿勢のz軸の傾き(角度) ※(ラジアン)でないことに注意
 * @param moving_time[uint16_t] 移動にかける時間(ms)
 */
void StewartPlatform::setIK(float pos_x, float pos_y, float pos_z, float rot_x, float rot_y, float rot_z, int16_t moving_time) {
  if (is_initialized_) {
    // 変更後のプラットフォームの中心位置を決める
    float center_position[3] = {pos_x, pos_y, pos_z + platform_z_home_};
    
    // プラットフォームの回転計算のため回転行列を作る
    float rotation_matrix[3][3];
    getMatrix_(radians(rot_x), radians(rot_y), radians(rot_z), rotation_matrix);
  
    // プラットフォームの各リンク接続部の座標を求める
    float target_position[6][3];
    getTargetPositions_(center_position, rotation_matrix, target_position);
  
    // 各サーボの角度を求める
    for (int8_t sid = 0; sid < 6; sid++) {
      sp_servo_[sid].setTargetAngle(calculateServoRotationAngle_(sid, BASE_LINK_POINTS[sid], target_position[sid]), moving_time);
    }
  }
}

/**
 * (private)プラットフォームのデフォルト高を計算する
 * 
 * @param link_length[float] リンクの長さをmmで
 * @return [float] プラットフォームのデフォルト高をmmで
 */
float StewartPlatform::calculatePlatformZHome_(float link_length) {
  // プラットフォームの中心位置を決める
  float center_position[3] = {0, 0, 0};
  
  // プラットフォームの回転計算のため回転行列を作る
  float rotation_matrix[3][3];
  getMatrix_(0, 0, 0, rotation_matrix);
  
  // プラットフォームの各リンク接続部の座標を求める
  float target_position[6][3];
  getTargetPositions_(center_position, rotation_matrix, target_position);

  float target_base_position[3];
  target_base_position[0] = SERVO_ARM_LENGTH * cos(SERVO_ARM_DIRECTIONS[0]) + BASE_LINK_POINTS[0][0]; // x
  target_base_position[1] = SERVO_ARM_LENGTH * sin(SERVO_ARM_DIRECTIONS[0]) + BASE_LINK_POINTS[0][1]; // y
  target_base_position[2] = 0; // z
  // ベースとプラットフォームの接続点間の距離計算
  float xd = target_position[0][0] - target_base_position[0];
  float yd = target_position[0][1] - target_base_position[1];
  float zd = sqrt(link_length * link_length - xd * xd - yd * yd);
  return zd;
}

/**
 * (Private)回転行列を求める関数
 * 
 * @param rot_x[float] x軸の回転(ラジアン)
 * @param rot_y[float] y軸の回転(ラジアン)
 * @param rot_z[float] z軸の回転(ラジアン)
 * @param matrix[float[3][3]] 値戻し用の配列のポインタ渡し 
 */
void StewartPlatform::getMatrix_(float rot_x, float rot_y, float rot_z, float matrix[3][3]) {
  matrix[0][0] = cos(rot_z) * cos(rot_y);
  matrix[0][1] = -sin(rot_z) * cos(rot_x) + cos(rot_z) * sin(rot_y) * sin(rot_x);
  matrix[0][2] = sin(rot_z) * sin(rot_x) + cos(rot_z) * cos(rot_x) * sin(rot_y);
  matrix[1][0] = sin(rot_z) * cos(rot_y);
  matrix[1][1] = cos(rot_z) * cos(rot_x) + sin(rot_z) * sin(rot_y) * sin(rot_x);
  matrix[1][2] = cos(rot_y) * sin(rot_x);
  matrix[2][0] = -sin(rot_y);
  matrix[2][1] = -cos(rot_z) * sin(rot_x) + sin(rot_z) * sin(rot_y) * cos(rot_x);
  matrix[2][2] = cos(rot_y) * cos(rot_x);
}

/**
 * (Private)プラットフォーム変化後の各リンク接続点の位置を求める
 * 
 * @param center[float[3]] プラットフォームの移動先中心点
 * @param matrix[float[3][3]]  プラットフォームの回転行列
 * @param positions[float[6][3]] 値を戻す用のプラットフォームのリンクとの接続点の配列
 */
void StewartPlatform::getTargetPositions_(float center[3], float matrix[3][3], float positions[6][3]) {
  for (int i = 0; i < 6; i++) {
    positions[i][0] = center[0] + matrix[0][0] * (PLATFORM_LINK_POINTS[i][0]) + matrix[1][0] * (PLATFORM_LINK_POINTS[i][1]) + matrix[2][0] * (PLATFORM_LINK_POINTS[i][2]); // x
    positions[i][1] = center[1] + matrix[0][1] * (PLATFORM_LINK_POINTS[i][0]) + matrix[1][1] * (PLATFORM_LINK_POINTS[i][1]) + matrix[2][1] * (PLATFORM_LINK_POINTS[i][2]); // y
    positions[i][2] = center[2] + matrix[0][2] * (PLATFORM_LINK_POINTS[i][0]) + matrix[1][2] * (PLATFORM_LINK_POINTS[i][1]) + matrix[2][2] * (PLATFORM_LINK_POINTS[i][2]); // z
  }
}

/**
 * (private)サーボの回転角度を求める
 * 
 * @param sid[int8_t] サーボのID
 * @param base_position[const float[2]] ベースの接続点のXY座標の配列
 * @param platform_position[float[3]] プラットフォームの接続点の目標点
 * @return [float] サーボの回転角度(ラジアン)
 */
float StewartPlatform::calculateServoRotationAngle_(int8_t sid, const float base_position[2], float platform_position[3]) {
  int counter = 0;
  float angle = 0;
  float target_base_position[3];
  float d[3];
  float distance;
  float gap;
  float threshold = 0.001;
  double min = SERVO_ROTATION_MIN;
  double max = SERVO_ROTATION_MAX;

  angle = sp_servo_[sid].getCurrentAngle();
  while (counter < 25) {
    // ベースの接続点の計算
    target_base_position[0] = SERVO_ARM_LENGTH * cos(angle) * cos(SERVO_ARM_DIRECTIONS[sid]) + base_position[0]; // x
    target_base_position[1] = SERVO_ARM_LENGTH * cos(angle) * sin(SERVO_ARM_DIRECTIONS[sid]) + base_position[1]; // y
    target_base_position[2] = SERVO_ARM_LENGTH * sin(angle); // z
    // ベースとプラットフォームの接続点間の距離計算
    d[0] = platform_position[0] - target_base_position[0]; // x
    d[1] = platform_position[1] - target_base_position[1]; // y
    d[2] = platform_position[2] - target_base_position[2]; // z
    distance = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    // 距離とアーム長が（ほぼ）等しくなれば、それはその角度が正しいと言える
    gap = link_length_ - distance;
    if ((gap > -threshold) && (gap < threshold)) {
      return angle;
    }
    // 等しくないときは、行くべき方向の限界までの中点をゴールとして再計算（そのために数値を設定）
    if (distance < link_length_) {
      max = angle;
    } else {
      min = angle;
    }
    counter += 1;
    // 限界に到達すればそこで終了
    if (max == radians(SERVO_ROTATION_MIN) || min == radians(SERVO_ROTATION_MAX)) {
      return angle;
    }
    angle = min + (max - min) / 2;
  }
  return angle;
}


/**
 * ServoControlerクラスのコンストラクタ
 */
ServoControler::ServoControler() {
  current_angle_ = 0;
  target_angle_ = 0;
  remaining_count_ = 0;
}

/**
 * サーボパラメータの設定
 * 
 * @param vsid[uint8_t] V-duinoにおけるサーボ番号
 * @param reverse[int8_t] サーボの反転設定
 * @param offset[int16_t] サーボのオフセット設定
 * @param angle_min[float] サーボの最小角度
 * @param angle_max[float] サーボの最大角度
 * @param fps[float] サーボのFPS
 */
void ServoControler::setParameters(uint8_t vsid, int8_t reverse, int16_t offset, float angle_min, float angle_max, float fps) {
  vsid_ = vsid;
  reverse_ = reverse;
  offset_ = offset;
  angle_min_ = angle_min;
  angle_max_ = angle_max;
  fps_ = fps;
}

/**
 * サーボの状態を変化させる
 */
void ServoControler::changeState() {
  float next_angle;
  if (remaining_count_ > 0) {
    next_angle = current_angle_ + ((target_angle_ - current_angle_) / remaining_count_);
    remaining_count_--;
    current_angle_ = next_angle;
  }
}
/**
 * サーボの目標角度を設定する
 * 
 * @param angle[float] 
 * @param moving_time[uint16_t] 
 */
void ServoControler::setTargetAngle(float angle, uint16_t moving_time) {
  target_angle_ = constrain(angle, angle_min_, angle_max_);
  remaining_count_ = (int)(moving_time * fps_ / 1000);
  if (remaining_count_ == 0) {
    remaining_count_ = 1;
  }
}

/**
 * vsid_のgetter
 * 
 * @return [uint8_t] vsid_
 */
uint8_t ServoControler::getVsid() {
  return vsid_;
}

/**
 * reverse_のgetter
 * 
 * @return [int8_t] reverse_
 */
int8_t ServoControler::getReverse() {
  return reverse_;
}

/**
 * offset_のgetter
 * 
 * @return [uint16_t] offset_
 */
int16_t ServoControler::getOffset() {
  return offset_;
}

/**
 * current_angle_のgetter
 * 
 * @return [float] current_angle_
 */
float ServoControler::getCurrentAngle() {
  return current_angle_;
}

/**
 * target_angle_のgetter
 * 
 * @return [float] target_angle_
 */
float ServoControler::getTargetAngle() {
  return target_angle_;
}
