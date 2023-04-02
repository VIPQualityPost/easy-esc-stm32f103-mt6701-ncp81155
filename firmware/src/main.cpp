#include <Arduino.h>
#include <SimpleFOC.h>
#include <MT6701_I2C.h>
#include "stm32f1xxMT6071_NCP81155.h"

#define ENC_CPR 1024

#define POLEPAIRS 4
#define Rphase 1.75
#define MOTOR_KV 1000

#ifdef MT6701_ABZ
Encoder mt6701 = Encoder(ENC_A, ENC_B, ENC_CPR, ENC_Z);

// interrupt handlers
void doA(){
  mt6701.handleA();
}

void doB(){
  mt6701.handleB();
}

void doZ(){
  mt6701.handleIndex();
}
#endif

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

#ifdef MT6701_SSI
// SSI code
#endif

BLDCDriver3PWM driver =  BLDCDriver3PWM(PWM_U, PWM_V, PWM_W, EN_U, EN_V, EN_W);
BLDCMotor motor = BLDCMotor(POLEPAIRS,Rphase,MOTOR_KV);
// HardwareSerial stlinkSerial(UART1_RX,UART1_TX);

void mt6701_i2c_enable(bool state){
  digitalWrite(ENC_MODE,state);
  digitalWrite(ENC_I2C_EN,state);
}

void setup(){

  SerialUSB.begin(115200);
  SimpleFOCDebug::enable(&SerialUSB);

  pinMode(ENC_MODE,OUTPUT);
  pinMode(ENC_I2C_EN,OUTPUT);

  #ifdef MT6701_ABZ
  // set the MT6701 to ABZ mode
  mt6701_i2c_enable(0);
  delay(1000);
  // Initialize after letting settle briefly (600ms?)
  mt6701.quadrature = Quadrature::OFF;
  mt6701.init(); 
  mt6701.enableInterrupts(doA,doB,doZ);
  motor.linkSensor(&mt6701);
  #endif

  #ifdef MT6701_I2C
  // set the MT6701 to serial mode
  mt6701_i2c_enable(1);
  delay(1000); //let chip settle
  mt6701.init(&enc_i2c);
  motor.linkSensor(&mt6701);
  #endif

  #ifdef MT6701_SSI
  // SSI init code
  #endif

  // setup the driver
  driver.pwm_frequency = 25000;
  driver.voltage_power_supply = 5;
  driver.voltage_limit  = 5;
  driver.init();
  motor.linkDriver(&driver);
  SerialUSB.println("Driver ready.");

  // closed loop parameters
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01;

  motor.P_angle.P = 15;
  motor.P_angle.I = 0.1;
  motor.P_angle.D = 0;
  motor.P_angle.output_ramp = 10000; //rad/s^2
  motor.LPF_angle.Tf = 0; //try to avoid

  // motor parameters
  motor.current_limit = 0.5;
  motor.velocity_limit = 20;
  motor.controller = MotionControlType::angle;
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.init();
  motor.initFOC();
  SerialUSB.println("Motor initalized.");

  motor.target = 2;

}

void loop() {

  motor.loopFOC();
  motor.move();

}