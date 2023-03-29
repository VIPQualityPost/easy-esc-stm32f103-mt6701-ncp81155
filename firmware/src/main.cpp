#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "./stm32f1xxMT6071_NCP81155.h"

#define POLEPAIRS 7 

Encoder mt6701 = Encoder(ENC_A, ENC_B, ENC_CPR, ENC_Z);
BLDCDriver3PWM driver =  BLDCDriver3PWM(PWM_U, PWM_V, PWM_W, EN_U, EN_V, EN_W);
BLDCMotor motor = BLDCMotor(POLEPAIRS);

//Commander commander = Commander(Serial);

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

// Commander interface
//void doMotor(char* cmd){
  //commander.motor(&motor,cmd);
//}

void setup() {
  
  mt6701.init(); 
  mt6701.enableInterrupts(doA,doB,doZ);
  motor.linkSensor(&mt6701);

  driver.pwm_frequency = 25000;
  driver.voltage_power_supply = 5;
  driver.voltage_limit  = 5;
  driver.init();
  motor.linkDriver(&driver);

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01;

  //motor.P_angle.P = 20;
  //motor.P_angle.I = 0;
  //motor.P_angle.D = 0;
  //motor.P_angle.output_ramp = 10000; //rad/s^2
  //motor.LPF_angle.Tf = 0; //try to avoid

  motor.current_limit = 0.5;
  motor.velocity_limit = 40;
  motor.controller = MotionControlType::velocity;

  motor.init();
  motor.initFOC();

  SerialUSB.begin();
  SerialUSB.println("Motor ready!");

  //commander.add('M', doMotor, "motor");

  delay(500);
}

void loop() {

  motor.loopFOC();
  motor.move(2);

  //commander.run();
}