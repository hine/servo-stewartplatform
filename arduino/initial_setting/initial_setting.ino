#include <vs-rc202.h>
#include <EEPROM.h>

// EEPROM(ESPなので厳密には違うけど)に書き込む個体差情報のための構造体
struct RobotBodyConfig {
  char header1;
  char header2;
  int16_t servo1_offset; // サーボ1のオフセット値を-500～500で
  int16_t servo2_offset; // サーボ2のオフセット値を-500～500で
  int16_t servo3_offset; // サーボ3のオフセット値を-500～500で
  int16_t servo4_offset; // サーボ4のオフセット値を-500～500で
  int16_t servo5_offset; // サーボ5のオフセット値を-500～500で
  int16_t servo6_offset; // サーボ6のオフセット値を-500～500で
  float link_length;     // リンクの有効長をmmで
};

// EEPROMへのデータ保存用のデータ変数
RobotBodyConfig robot_body_config;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("");
  Serial.println("");

  Serial.println("=== Stewart Platform Robot Initial Setting ===");  
  Serial.println("");

  // EEPROMからデータの読み出し
  EEPROM.begin(20);
  EEPROM.get<RobotBodyConfig>(0, robot_body_config);

  // EEPROMからのデータのヘッダを確認
  if (robot_body_config.header1 == 's' && robot_body_config.header2 == 'p') {
    Serial.println("Found current config.");
    Serial.print("Servo1 Offset: ");
    Serial.println(robot_body_config.servo1_offset);
    Serial.print("Servo2 Offset: ");
    Serial.println(robot_body_config.servo2_offset);
    Serial.print("Servo3 Offset: ");
    Serial.println(robot_body_config.servo3_offset);
    Serial.print("Servo4 Offset: ");
    Serial.println(robot_body_config.servo4_offset);
    Serial.print("Servo5 Offset: ");
    Serial.println(robot_body_config.servo5_offset);
    Serial.print("Servo6 Offset: ");
    Serial.println(robot_body_config.servo6_offset);
    Serial.println("-");
    Serial.print("Link Length: ");
    Serial.println(robot_body_config.link_length);
    Serial.println("");
  }
  
  Serial.println("-- Servo Offset --");
  Serial.print("Please wait...");

  // VS-RC202の初期化
  initLib();

  // サーボの限界値を設定（-90度～90度）
  setServoLimitL(-1800);
  setServoLimitH(1800);

  for (int servo_id = 1; servo_id <= 6; servo_id++) {
    setServoOffset(servo_id, 0);
    servoEnable(servo_id, 1);
  }    
  setServoMovingTime(100);
  setServoMode(0);
  for (int servo_id = 1; servo_id <= 6; servo_id++) {
    setServoDeg(servo_id, 0);
  }
  moveServo();

  Serial.println("done.");
  Serial.println("Please set servo horn.");
  Serial.println("");

  int servo_offsets[6] = {0, 0, 0, 0, 0, 0};

  bool go_next;
  int servo_id;

  // サーボオフセット値の設定
  for (int servo_id = 1; servo_id <= 6; servo_id++) {
    go_next = false;
    while (!go_next) {
      Serial.print("Servo");
      Serial.print(servo_id);
      Serial.print(" Offset ? ");
      while (Serial.available() == 0) {
        delay(1);
      }
      delay(10);
      servo_offsets[servo_id - 1] = Serial.parseInt();
      Serial.println(servo_offsets[servo_id - 1]);
      while (Serial.available()) {
        Serial.read();
      }
  
      // 新しいオフセットでサーボのゼロ位置の移動
      setServoOffset(servo_id, servo_offsets[servo_id - 1]);
      setServoDeg(servo_id, 0);
      moveServo();
  
      Serial.print("Please check Servo");
      Serial.print(servo_id);
      Serial.print(" angle. Is it OK? (y/n) ? ");
      while(Serial.available()==0) {
        delay(1);
      }
      if (Serial.read() == 'y') {
        Serial.println("y");
        go_next = true;
      } else {
        Serial.println("n");
      }
      while (Serial.available()) {
        Serial.read();
      }
    }
    Serial.println("");
  }
    
  // リンク長の設定
  Serial.print("Link Length ? ");
  while(Serial.available()==0) {
    delay(1);
  }
  delay(10);
  robot_body_config.link_length = Serial.parseFloat();
  Serial.println(robot_body_config.link_length);
  Serial.println("");
  while (Serial.available()) {
    Serial.read();
  }

  robot_body_config.servo1_offset = servo_offsets[0];
  robot_body_config.servo2_offset = servo_offsets[1];
  robot_body_config.servo3_offset = servo_offsets[2];
  robot_body_config.servo4_offset = servo_offsets[3];
  robot_body_config.servo5_offset = servo_offsets[4];
  robot_body_config.servo6_offset = servo_offsets[5];

  // ヘッダの定義
  robot_body_config.header1 = 's';
  robot_body_config.header2 = 'p';

  // EEPROMへデータ書き込み
  EEPROM.begin(20);
  EEPROM.put<RobotBodyConfig>(0, robot_body_config);
  EEPROM.commit();

  Serial.println("Done.");
}

void loop() {
  
}
