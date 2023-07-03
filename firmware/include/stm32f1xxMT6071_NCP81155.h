#pragma once

// encoder interface
#define ENC_A PC13
#define ENC_B PC14
#define ENC_Z PC15

#define ENC_PUSH PB1
#define ENC_MODE PB0
#define ENC_I2C_EN PB2

// mosfet driver pins
// allow for dir switch to match encoder on build flag MOTOR_CW

#define PWM_U PA9
#define EN_U PB14
#define PWM_V PA8
#define EN_V PB13  
#define PWM_W PA10
#define EN_W PB15 

// current sensing pins
#define CUR_U PA1
#define CUR_V PA0
#define CUR_W PA4

// ws2812E
#define RGB_LED PA15

// SPI1
#define SPI1_MOSI PA7
#define SPI1_MISO PA6
#define SPI1_SCLK PA5

// UART1
#define UART1_TX PB6
#define UART1_RX PB7

// CANBUS1
#define CAN_TX PB9
#define CAN_RX PB8 

// I2C2
#define I2C2_SDA PB11
#define I2C2_SCL PB10

// UART2
#define UART2_TX PA2
#define UART2_RX PA3



