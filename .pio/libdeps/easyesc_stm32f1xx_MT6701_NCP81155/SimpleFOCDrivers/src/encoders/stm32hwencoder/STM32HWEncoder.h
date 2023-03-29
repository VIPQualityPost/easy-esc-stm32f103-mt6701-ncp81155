
#ifndef STM32_HARDWARE_ENCODER_H
#define STM32_HARDWARE_ENCODER_H

#include <Arduino.h>


#if defined(_STM32_DEF_)

#include <HardwareTimer.h>
#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"

class STM32HWEncoder : public Sensor {
  public:
    /**
    Encoder class constructor
    @param ppr  impulses per rotation  (cpr=ppr*4)
    */
    explicit STM32HWEncoder(unsigned int ppr, int8_t pinA, int8_t pinB, int8_t pinI=-1);

    /** encoder initialise pins */
    void init() override;

    // Encoder configuration
    unsigned int cpr;  //!< encoder cpr number

    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float getSensorAngle() override;
    float getMechanicalAngle() override;
    /**  get current angular velocity (rad/s) */
    float getVelocity() override;
    float getAngle() override;
    double getPreciseAngle() override;
    int32_t getFullRotations() override;
    void update() override;

    /**
     * returns 0 if it does need search for absolute zero
     * 0 - encoder without index
     * 1 - ecoder with index
     */
    int needsSearch() override;

  private:
    int hasIndex();  // !< function returning 1 if encoder has index pin and 0 if not.

    void handleOverflow();

    TIM_HandleTypeDef encoder_handle;

    static constexpr u_int16_t overflow_margin = 20000;
    u_int16_t rotations_per_overflow;
    u_int16_t ticks_per_overflow;

    volatile int32_t overflow_count;
    volatile u_int16_t count;  //!< current pulse counter
    volatile u_int16_t prev_count;
    volatile int32_t prev_overflow_count;

    // velocity calculation variables
    volatile int32_t pulse_timestamp, prev_timestamp;

    int8_t _pinA, _pinB, _pinI;
};

#endif
#endif