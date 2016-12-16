#ifndef HXSERVO_h
#define HXSERVO_h

#include "Arduino.h"

#define USART_2_WIRE     0x01 ///双线串口模式
#define USART_1_WIRE     0x02 ///单线串口模式
#define I2C              0x03 ///I2C模式

class HXZXServo
{
 public:
  const static uint8_t SETTING_OK    =   0x01; ///设置寄存器数据成功 只能从机发送
  const static uint8_t SETTING_FAIL  =   0x02; ///设置寄存器数据失败 只能从机发送

  const static uint8_t READ_REG    = 0x01; ///读寄存器数据 只能从机发送
  const static uint8_t WRITE_REG   = 0x02; ///写寄存器数据 只能主机发送
  const static uint8_t ANSWER      = 0x03; ///舵机应答 只能舵机发送

  const static uint8_t POSITION_REGADDR = 0x01; ///位置寄存器 可读写
  const static uint8_t SPEED_REGADDR    = 0x02; ///速度寄存器 可读写
  const static uint8_t ADDR_REGADDR     = 0x03; ///舵机在通信总线上的地址寄存器 可读写
  const static uint8_t BAUDRATE_REGADDR = 0x04; ///波特率寄存器 可读写
  const static uint8_t COMMUNICATION_MODE_REGADDR = 0x21; ///通信模式寄存器 可读写
  const static uint8_t P_GAIN_REGADDR   = 0x05; ///P参数寄存器 可读写
  const static uint8_t D_GAIN_REGADDR   = 0x06; ///D参数寄存器 可读写
  const static uint8_t I_GAIN_REGADDR   = 0x07; ///I参数寄存器 可读写
  const static uint8_t PDI_GAIN_REGADDR = 0x08; ///PDI参数寄存器 可读写
  const static uint8_t PDI_DEADBAND_REGADDR        = 0x1f; ///PDI死区范围寄存器   可读写
  const static uint8_t ACCELERATION_REGADDR        = 0x20; ///加速度寄存器   可读写
  const static uint8_t CW_FLEXIBLE_MARGIN_REGADDR  = 0x09; ///顺时针柔性边距寄存器 可读写
  const static uint8_t CCW_FLEXIBLE_MARGIN_REGADDR = 0x0a; ///逆时针柔性边距寄存器 可读写
  const static uint8_t CW_FLEXIBLE_SLOPE_REGADDR   = 0x0b; ///顺时针柔性斜率寄存器 可读写
  const static uint8_t CCW_FLEXIBLE_SLOPE_REGADDR  = 0x0c; ///逆时针柔性斜率寄存器 可读写
  const static uint8_t WORKING_MODE_REGADDR    = 0x0d; ///工作模式寄存器
  const static uint8_t SERVO_STATE_REGADDR     = 0x0e; ///舵机状态寄存器 可读写
  const static uint8_t MAX_POSITION_REGADDR    = 0x0f; ///最大位置寄存器 可读写
  const static uint8_t MIN_POSITION_REGADDR    = 0x10; ///最小位置寄存器 可读写
  const static uint8_t MAX_TEMPERATURE_REGADDR = 0x11; ///最高温度寄存器 可读写
  const static uint8_t MAX_VOLTAGE_REGADDR     = 0x12; ///最大电压寄存器 可读写
  const static uint8_t MIN_VOLTAGE_REGADDR     = 0x13; ///最小电压寄存器 可读写
  const static uint8_t MAX_TORQUE_REGADDR      = 0x14; ///最大扭矩寄存器 可读写
  const static uint8_t ALL_WRITEREAD_REGADDR   = 0x15; ///所有可读写寄存器地址 可读写

  const static uint8_t CURRENT_POSITION_REGADDR    =   0x16; ///当前位置寄存器 可读
  const static uint8_t CURRENT_SPEED_REGADDR       =   0x17; ///当前速度寄存器 可读
  const static uint8_t CURRENT_TORQUE_REGADDR      =   0x18; ///当前扭矩寄存器 可读
  const static uint8_t CURRENT_VOLTAGE_REGADDR     =   0x19; ///当前电压寄存器 可读
  const static uint8_t CURRENT_TEMPERATURE_REGADDR =   0x1a; ///当前温度寄存器 可读
  const static uint8_t ALL_READONLY_REGADDR        =   0x1b; ///所有只读寄存器地址 可读

  const static uint8_t FIRMWARE_VERSION_REGADDR    =   0x1c; ///固件版本 可读
  const static uint8_t MODEL_CODE_REGADDR          =   0x1d; ///舵机型号代码 可读
  const static uint8_t WRITE_FLASH_REGADDR         =   0x1e; ///写FLASH寄存器地址 可写

  const static uint16_t MASTER_BUSADDR    =  0x0001; ///主机在通信总线上的地址
  const static uint16_t BROADCAST_BUSADDR =  1000; ///广播地址

  HXZXServo();
  HXZXServo(HardwareSerial *serial1,uint16_t baud);
  HXZXServo(HardwareSerial *serial1,uint16_t baud, uint8_t txEn, uint8_t rxEn);
  uint16_t readServo(uint16_t devAddr,uint8_t regAddr);
  uint8_t writeServo(uint16_t devAddr,uint8_t regAddr,uint16_t data);
  void changeToI2C();
  void changeToSerial(HardwareSerial *serial1,uint16_t baud);
  void changeToSerial(HardwareSerial *serial1,uint16_t baud, uint8_t txEn, uint8_t rxEn);
  void changeSerialTimeout(uint16_t tim);

private:
  uint8_t Mode;
  uint16_t Baudrate;
  HardwareSerial *serial;
  uint8_t TxEn;
  uint8_t RxEn;
};

#endif
