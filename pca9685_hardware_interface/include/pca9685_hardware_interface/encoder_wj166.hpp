
#ifndef __ENCODER_WJ166_HPP__
#define __ENCODER_WJ166_HPP__

#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <cmath>

#include <modbus/modbus.h>

namespace encoder_wj166 {

class Implementation
{
 public:
  Implementation(const char *device_path = "/dev/ttyUSB0");
  virtual ~Implementation();
  
  double get_velocity(const int channel); // 获取速度 单位 rad/s
  double get_position(const int channel); // 获取位置 单位 rad
  void   clear_entry(const int channel);  // 清零指定编码器值
  void   clear_all();                     // 清零所有编码器值
  
 private:
  void run_();
  void stop_();
  
  void init_encoder_();

  void get_position_real_();
  void get_velocity_real_();

  double encoder_readings_per_second_{100};

  std::shared_ptr<std::thread> thread_;
  volatile bool do_stop_{false};
  std::chrono::duration<double, std::ratio<1L>> period_;
  
  std::mutex readings_mutex_;
  double position_last_{0.0};
  double velocity_last_{0.0};

  double velocity_last_position_{0.0};
  std::chrono::high_resolution_clock::time_point velocity_last_time_;

  modbus_t *ctx_{NULL};
};

}  // namespace encoder_wj166

#endif  // __ENCODER_WJ166_HPP__
