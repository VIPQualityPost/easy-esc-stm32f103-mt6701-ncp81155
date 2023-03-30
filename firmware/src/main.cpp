#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "./stm32f1xxMT6071_NCP81155.h"

#define POLEPAIRS 7 
#define BAUDRATE 115200

Encoder mt6701 = Encoder(ENC_A, ENC_B, ENC_CPR, ENC_Z);
BLDCDriver3PWM driver =  BLDCDriver3PWM(PWM_U, PWM_V, PWM_W, EN_U, EN_V, EN_W);
BLDCMotor motor = BLDCMotor(POLEPAIRS,8,600);

//interrupt handlers

void doA(){
  mt6701.handleA();
}

void doB(){
  mt6701.handleB();
}

void doZ(){
  mt6701.handleIndex();
}

void setup(){

  SimpleFOCDebug::enable();

  SerialUSB.begin();
  SerialUSB.dtr(false);

  // set the MT6701 to ABZ mode
  pinMode(ENC_A,OUTPUT);
  digitalWrite(ENC_MODE,0);

  mt6701.init(); 
  mt6701.enableInterrupts(doA,doB,doZ);
  motor.linkSensor(&mt6701);
  SerialUSB.println("Encoder ready.");

  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 5;
  driver.voltage_limit  = 5;
  driver.init();
  motor.linkDriver(&driver);

  SerialUSB.println("Driver ready.");

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01;

  motor.P_angle.P = 20;
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;
  motor.P_angle.output_ramp = 10000; //rad/s^2
  motor.LPF_angle.Tf = 0; //try to avoid

  motor.voltage_limit = 5;
  motor.velocity_limit = 40;
  motor.controller = MotionControlType::angle;
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.init();
  motor.initFOC();
  SerialUSB.println("Motor ready.");

  motor.target = 20;
}

void loop() {

  motor.loopFOC();
  motor.move();

}