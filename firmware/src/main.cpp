#include <Arduino.h>
#include <SimpleFOC.h>
#include <MT6701_I2C.h>
#include "stm32f1xxMT6071_NCP81155.h"
// #include <RTTStream.h>

// Motor specific parameters
#define POLEPAIRS 4
#define Rphase 1.75
#define MOTOR_KV 1000

/**
 * Magnetic sensor configuration schemes.
 * Set using build flag -DMT6701_ABZ, -DMT6701_I2C, -DMT6701_SSI.
*/

#ifdef MT6701_I2C
MT6701_I2CConfig_s mt6701_config = {
  .chip_address = 0b0000110, 
  .bit_resolution = 14, 
  .angle_register=0x03, 
  .data_start_bit= 8
  }; 

MT6701_Serial_I2C mt6701 = MT6701_Serial_I2C(mt6701_config);
TwoWire enc_i2c(I2C2_SDA, I2C2_SCL);
#endif

// Prepare SimpleFOC constructors.
BLDCDriver3PWM driver =  BLDCDriver3PWM(PWM_U, PWM_V, PWM_W, EN_U, EN_V, EN_W);
BLDCMotor motor = BLDCMotor(POLEPAIRS,Rphase,MOTOR_KV);
// HardwareSerial stlinkSerial(UART1_RX,UART1_TX);
// RTTStream rtt;

#ifdef HAS_COMMANDER
Commander commander = Commander(SerialUSB);
void doMotor(char *cmd){
  commander.motor(&motor,cmd);
}
#endif

void mt6701_i2c_enable(bool state){
  digitalWrite(ENC_MODE,state);
  digitalWrite(ENC_I2C_EN,state);
}

void setup(){

  #ifdef SIMPLEFOC_STM32_DEBUG
  SimpleFOCDebug::enable(&SerialUSB);
  #endif

  pinMode(ENC_MODE,OUTPUT);
  pinMode(ENC_I2C_EN,OUTPUT);

  #ifdef MT6701_I2C
  // set the MT6701 to serial mode
  mt6701_i2c_enable(1);
  delay(1000); //let chip settle
  mt6701.init(&enc_i2c);
  motor.linkSensor(&mt6701);
  #endif

  // setup the driver
  driver.pwm_frequency = 25000;
  driver.voltage_power_supply = 5;
  driver.voltage_limit  = 5;
  driver.init();
  motor.linkDriver(&driver);

  // closed loop parameters
  motor.PID_velocity.P = 1;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0.005;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 1;

  motor.P_angle.P = 1;
  motor.P_angle.I = 10;
  // motor.P_angle.D = 0;
  // motor.P_angle.output_ramp = 1000; //rad/s^2
  motor.LPF_angle.Tf = 0; //try to avoid

  // motor parameters
  motor.voltage_sensor_align = 2;
  motor.current_limit = 0.5;
  motor.velocity_limit = 20;
  motor.controller = MotionControlType::angle;
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.init();
  motor.initFOC();

  // Commander actions
  #ifdef HAS_COMMANDER
  SerialUSB.begin();
  motor.useMonitoring(SerialUSB);
  motor.monitor_start_char = 'M';
  motor.monitor_end_char = 'M';
  motor.monitor_downsample = 250;
  commander.add('M',doMotor,"motor");
  commander.verbose = VerboseMode::machine_readable;
  #endif

  // rtt.println("setup done!");
  motor.target = 2;

}

void loop() {

  motor.loopFOC();
  motor.move();

  #ifdef HAS_COMMANDER
  // motor.monitor();
  commander.run();
  #endif

}