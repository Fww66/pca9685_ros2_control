
#include "pca9685_hardware_interface/encoder_wj166.hpp"

#include <system_error>
#include <string>
#include <cmath>

namespace encoder_wj166 {

Implementation::Implementation(const char *device_path /*= "/dev/ttyUSB0"*/)
{
  ctx_ = modbus_new_rtu(device_path, 9600, 'N', 8, 1);
  if (NULL == ctx_)                //对应的设备描述符为ttyUSB0
  {
    throw std::system_error(errno, std::system_category(), "Unable to allocate libmodbus contex.");
  }
  if (-1 == modbus_connect(ctx_)) //等待连接设备
  {
    const auto msg = "Connection failed: " + std::string(modbus_strerror(errno));
    throw std::system_error(errno, std::system_category(), msg);
  }

  modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485);    //set COM mode
  modbus_set_slave(ctx_, 1);      //set slave ID
  modbus_set_debug(ctx_, 0);      //设置1可看到调试信息  
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
  uint16_t tab_reg[32] = { 0 };
  modbus_read_registers(ctx_, 0x0, 32, tab_reg); // read all the data

  int32_t encoder[16] = { 0 };
  for (int j = 0; j < 16; j++)
  {
    encoder[j] = tab_reg[j*2+1] << 16 | tab_reg[j*2];
  }

  for (int j = 0; j < 4; j++)
  { 
    velocity_[j] = (double)encoder[12+j] * 2 * M_PI / 60.0; 
    position_[j] = (double)encoder[j] * 2 * M_PI / 6400.0;
  }  
}

double Implementation::get_velocity(const int id)
{
  return velocity_[id];
}

double Implementation::get_position(const int id)
{
  return position_[id];
}

void Implementation::clear_entry(const int id)  // 清零指定编码器值
{
  modbus_write_register(ctx_, 0x43, 0x0A + id);   // 清零寄存器 10(0x0A)为清零编码器0
}

void Implementation::clear_all()                     // 清零所有编码器值
{
  modbus_write_register(ctx_, 0x43, 0x12);        //18(0x12)为所有寄存器
}

}  // namespace encoder_wj166
