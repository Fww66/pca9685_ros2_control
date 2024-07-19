
#ifndef __ENCODER_WJ166_HPP__
#define __ENCODER_WJ166_HPP__

#include <modbus/modbus.h>

namespace encoder_wj166 {

class Implementation
{
 public:
  Implementation(const char *device_path = "/dev/ttyUSB0");
  virtual ~Implementation();
  
  void   update();                     // 从编码器WJ166中读取所有位置速度
  double get_velocity(const int id);   // 获取速度 单位 rad/s
  double get_position(const int id);   // 获取位置 单位 rad
  void   clear_entry(const int id);    // 清零指定编码器值
  void   clear_all();                  // 清零所有编码器值
  
 private:
  modbus_t *ctx_{NULL};

  double position_[4] {0.0};
  double velocity_[4] {0.0};
};

}  // namespace encoder_wj166

#endif  // __ENCODER_WJ166_HPP__
