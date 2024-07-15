
#include "pca9685_hardware_interface/encoder_wj166.hpp"

namespace encoder_wj166 {

Implementation::Implementation(const char *device_path /*= "/dev/ttyUSB0"*/)
{
  // ctx_ = modbus_new_rtu(device_path, 9600, 'N', 8, 1);
  // if (ctx_ == NULL)                //对应的设备描述符为ttyUSB0
  // {
  //   fprintf(stderr, "Unable to allocate libmodbus contex\n");
  //   return;
  // }

  init_encoder_();


}

Implementation::~Implementation()
{ 
  stop_(); 
}

double Implementation::get_velocity(const int id)
{
  readings_mutex_.lock();
  get_velocity_real_();
  double value = velocity_last_;
  readings_mutex_.unlock();

  return value;
}

double Implementation::get_position(const int id)
{
  readings_mutex_.lock();
  get_position_real_();
  double value = position_last_;
  readings_mutex_.unlock();

  return value;
}

void   Implementation::clear_entry(const int id)  // 清零指定编码器值
{
  
}

void   Implementation::clear_all()                     // 清零所有编码器值
{
  
}

void Implementation::run_()
{
  while (!do_stop_) 
  {
    std::this_thread::sleep_for(period_);
    
    readings_mutex_.lock();

    get_position_real_();
    get_velocity_real_();

    readings_mutex_.unlock();
  }

}

void Implementation::stop_()
{
  do_stop_ = true;
  if (thread_.get()) 
  {
    thread_->join();
    thread_.reset();
  }
}

void Implementation::init_encoder_()
{
  period_ = std::chrono::seconds(1) / encoder_readings_per_second_;
  thread_ = std::shared_ptr<std::thread>(new std::thread(&Implementation::run_, this));
}

void Implementation::get_position_real_()
{
  // modbus_read_registers
  position_last_ = 3.14; // 通过modbus从WJ166中读取编码器值，转换成弧度

}

void Implementation::get_velocity_real_()
{
  auto now = std::chrono::high_resolution_clock::now();
  auto time_delta = std::chrono::duration_cast<std::chrono::duration<double>>(now - velocity_last_time_);
  auto position_delta = position_last_ - velocity_last_position_;
  if (std::fabs(position_delta) > 0.5) 
  {
    // If this is a single turn encoder then,
    // for the sake of velocity calculation,
    // we need to compensate for the integer overrun (underrun).
    if (position_delta > 0) 
    {
      position_delta -= 1.0;
    } 
    else 
    {
      position_delta += 1.0;
    }
  }

  velocity_last_time_ = now;
  velocity_last_position_ = position_last_;

  velocity_last_ = position_delta / time_delta.count();
}

}  // namespace encoder_wj166
