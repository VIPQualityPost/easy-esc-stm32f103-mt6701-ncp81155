#pragma once

#ifndef HAPTICTEXTURE_H
#define HAPTICTEXTURE_H

#include <Arduino.h>
#include <SimpleFOC.h>

struct HapticConfig{ 
    uint8_t position;
    uint8_t min_position;
    uint8_t max_position;
    float position_width_rad;
    float detent_strength;
    float endstop_strength;
    float snap_point;
};

class HapticTexture : public BLDCMotor {
    public:
        /**
         * HapticTexture class constructor
         * @param *BLDCMotor object
         * @param 
        */
        HapticTexture(HapticConfig *_config, PIDController *_pidloop);

        /** 
         * Function for finding nearest detent
         * @param current_angle current position
        */
       float findAttractor(float current_angle);

        /** 
         * 
        */
    private: 


};


#endif