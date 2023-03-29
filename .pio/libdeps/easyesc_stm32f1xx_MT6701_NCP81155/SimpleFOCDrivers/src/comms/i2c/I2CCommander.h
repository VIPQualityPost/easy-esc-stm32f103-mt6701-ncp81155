
#ifndef SIMPLEFOC_I2CCOMMANDER_H
#define SIMPLEFOC_I2CCOMMANDER_H

#include "Arduino.h"
#include "Wire.h"
#include "common/base_classes/FOCMotor.h"
#include "I2CCommanderRegisters.h"

#ifndef I2CCOMMANDER_MAX_MOTORS_COMMANDABLE
#define I2CCOMMANDER_MAX_MOTORS_COMMANDABLE 4
#endif

#define I2CCOMMANDER_MIN_VELOCITY_FOR_MOTOR_MOVING 0.1f  // in rad/s
#define I2CCOMMANDER_MAX_REPORT_REGISTERS 8


class I2CCommander {
    public:    
        I2CCommander(TwoWire* wire = &Wire);
        ~I2CCommander();

        void addMotor(FOCMotor* motor);         // first add motors
        virtual void init(uint8_t address);     // then init

        void onReceive(int numBytes);
        void onRequest();

    protected:
        void writeFloat(float value);
        bool readBytes(void* valueToSet, uint8_t numBytes);
        virtual bool sendRegister(uint8_t motorNum, uint8_t registerNum);
        virtual bool receiveRegister(uint8_t motorNum, uint8_t registerNum, int numBytes);

        uint8_t _address;
        TwoWire* _wire;
        uint8_t numMotors = 0;
        uint8_t curMotor = 0;
        I2CCommander_Register curRegister = REG_STATUS;
        bool commanderror = false;
        bool lastcommanderror = false;
        uint8_t lastcommandregister = REG_STATUS;
        FOCMotor* motors[I2CCOMMANDER_MAX_MOTORS_COMMANDABLE];
        uint8_t numReportRegisters = 0;
        uint8_t reportMotors[I2CCOMMANDER_MAX_REPORT_REGISTERS];
        uint8_t reportRegisters[I2CCOMMANDER_MAX_REPORT_REGISTERS];        
};

#endif
