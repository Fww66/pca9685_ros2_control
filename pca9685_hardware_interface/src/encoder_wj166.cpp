
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

}

Implementation::~Implementation()
{
  /* Close the connection */
  if (ctx_)
  {
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = NULL;
  }
}

void   Implementation::update() // 更新位置速度，从编码器WJ166中读取
{
  
}

double Implementation::get_velocity(const int id)
{
  double value = 0.0;
  return value;
}

double Implementation::get_position(const int id)
{
  double value = 0.0;
  return value;
}

void   Implementation::clear_entry(const int id)  // 清零指定编码器值
{
  
}

void   Implementation::clear_all()                     // 清零所有编码器值
{
  
}

}  // namespace encoder_wj166
