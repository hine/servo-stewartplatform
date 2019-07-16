#include <sp_robot.h>
#include <Ticker.h>

Ticker ticker;

// シリアル通信のための設定
#define SERIAL_BAUDRATE 115200

// ロボットの状態変化のFPS
#define ROBOT_FPS 25

StewartPlatform sp;

void ticker_work() {
  sp.changeState();
}

void setup() {
  // スチュワートプラットフォームの初期化
  sp.init(ROBOT_FPS);

  // スチュワートプラットフォームの状態変化を定期的に行う
  ticker.attach_ms(1000 / ROBOT_FPS, ticker_work);

  // 初期姿勢
  sp.setIK(0, 0, 0, 0, 0, 0, 100);
  delay(1000);
}

void loop() {
  // XY軸の移動を円で
  for (int i = 0; i < 36; i++) {
    sp.setIK(50 * cos(radians(i * 10)), 50 * sin(radians(i * 10)), 0, 0, 0, 0, 100);
    delay(100);
  }
  sp.setIK(50, 0, 0, 0, 0, 0, 100);
  delay(100);
  sp.setIK(0, 0, 0, 0, 0, 0, 100);
  delay(1000);

  // Z軸の上げ下げ
  sp.setIK(0, 0, 30, 0, 0, 0, 100);
  delay(1000);
  sp.setIK(0, 0, 0, 0, 0, 0, 100);
  delay(1000);
  sp.setIK(0, 0, -30, 0, 0, 0, 100);
  delay(1000);
  sp.setIK(0, 0, 0, 0, 0, 0, 100);
  delay(1000);

  // XY軸の回転姿勢を円で  
  for (int i = 0; i < 36; i++) {
    sp.setIK(0, 0, 0, 30 * cos(radians(i * 10)), 30 * sin(radians(i * 10)), 0, 100);
    delay(100);
  }
  sp.setIK(0, 0, 0, 30, 0, 0, 100);
  delay(100);
  sp.setIK(0, 0, 0, 0, 0, 0, 100);
  delay(1000);

  // Z軸の回転
  sp.setIK(0, 0, 0, 0, 0, 30, 100);
  delay(1000);
  sp.setIK(0, 0, 0, 0, 0, 0, 100);
  delay(1000);
  sp.setIK(0, 0, 0, 0, 0, -30, 100);
  delay(1000);
  sp.setIK(0, 0, 0, 0, 0, 0, 100);
  delay(1000);
}
