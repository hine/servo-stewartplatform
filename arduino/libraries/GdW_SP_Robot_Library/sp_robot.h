/* sp_robot.h
 * Library for GdW's Stewart-platform Robot
 * Created by Daisuke IMAI <hine.gdw@gmail.com>
 * https://github.com/hine/servo-stewartplatform
 */
#ifndef sp_robot_h
#define sp_robot_h

#include <Arduino.h>

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

class ServoControler {
  public:
  private:
    uint8_t vsid_;
    int8_t reverse_;
    int16_t offset_;
    float angle_min_;
    float angle_max_;
    float fps_;
    float current_angle_;
    float target_angle_;
    int remaining_count_;
  public:
    ServoControler();
    void setParameters(uint8_t, int8_t, int16_t, float, float, float);
    void changeState();
    void setTargetAngle(float, uint16_t);
    uint8_t getVsid();
    int8_t getReverse();
    int16_t getOffset();
    float getCurrentAngle();
    float getTargetAngle();
  private:
};

class StewartPlatform {
  public:
  private:
    static const float SERVO_ROTATION_MIN;
    static const float SERVO_ROTATION_MAX;
    static const float RAD2DEG;
    static const float DEG30;
    static const float SERVO_ARM_DIRECTIONS[6];
    static const float ANGLE_BETWEEN_SERVOS;
    static const float ANGLE_GAP_TO_SERVO;
    static const float SERVO_ARM_LENGTH;
    static const float BASE_CENTER_TO_LINK_RADIUS;
    static const float PLATFORM_CENTER_TO_LINK_RADIUS;
    static const float ANGLE_BETWEEN_LINK_CONNECTIONS;
    static const float BASE_LINK_POINTS[6][2];
    static const float PLATFORM_LINK_POINTS[6][3];
    bool is_initialized_;
    float fps_;
    float link_length_;
    float platform_z_home_;
    float ik_data_[6];
    float servo_angle_[6];
    RobotBodyConfig robot_body_config_;
    ServoControler sp_servo_[6];
    int8_t vsid_table[11];
  public:
    StewartPlatform();
    void init(float);
    void changeState();
    void setServoAngle(uint8_t, float, uint16_t);
    void setIK(float, float, float, float, float, float, int16_t);
  private:
    float calculatePlatformZHome_(float);
    void getMatrix_(float, float, float, float [3][3]);
    void getTargetPositions_(float [3], float [3][3], float [6][3]);
    float calculateServoRotationAngle_(int8_t, const float[2], float[3]);
};
#endif
