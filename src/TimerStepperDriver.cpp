/*
 * Generic Stepper Motor Driver Driver
 * Indexer mode only.

 * Copyright (C)2015-2019 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 *
 * Linear speed profile calculations based on
 * - Generating stepper-motor speed profiles in real time - David Austin, 2004
 * - Atmel AVR446: Linear speed control of stepper motor, 2006
 */
#include "TimerStepperDriver.h"

/*
 * Min/Max functions which avoid evaluating the arguments multiple times.
 * See also https://github.com/arduino/Arduino/issues/2069
 */
template<typename T>
constexpr const T& stepperMin(const T& a, const T& b)
{
    return b < a ? b : a;
}

template<typename T>
constexpr const T& stepperMax(const T& a, const T& b)
{
    return a < b ? b : a;
}

/*
 * Timer connection: only DIR, STEP are connected.
 * Microstepping controls should be hardwired.
 */
TimerStepperDriver::TimerStepperDriver(short steps, short dir_pin, short step_pin)
:TimerStepperDriver(steps, dir_pin, step_pin, PIN_UNCONNECTED)
{
}

TimerStepperDriver::TimerStepperDriver(short steps, short dir_pin, short step_pin, short enable_pin)
:motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin), enable_pin(enable_pin)
{
	steps_to_cruise = 0;
	steps_remaining = 0;
	dir_state = 0;
	steps_to_brake = 0;
	step_pulse = 0;
    cruise_step_pulse = 0;
	rest = 0;
	step_count = 0;
}

/*
 * Initialize pins, calculate timings etc
 */
void TimerStepperDriver::begin(float rpm, short microsteps){
    pinMode(dir_pin, OUTPUT);
    digitalWrite(dir_pin, HIGH);

    pinMode(step_pin, OUTPUT);
    digitalWrite(step_pin, LOW);

    if IS_CONNECTED(enable_pin){
        pinMode(enable_pin, OUTPUT);
        disable();
    }

    this->rpm = rpm;
    setMicrostep(microsteps);

    enable();
}

/*
 * Set target motor RPM (1-200 is a reasonable range)
 */
void TimerStepperDriver::setRPM(float rpm){
    if (this->rpm == 0){        // begin() has not been called (old 1.0 code)
        begin(rpm, microsteps);
    }
    this->rpm = rpm;
}

/*
 * Set stepping mode (1:microsteps)
 * Allowed ranges for TimerStepperDriver are 1:1 to 1:128
 */
short TimerStepperDriver::setMicrostep(short microsteps){
    for (short ms=1; ms <= getMaxMicrostep(); ms<<=1){
        if (microsteps == ms){
            this->microsteps = microsteps;
            break;
        }
    }
    return this->microsteps;
}

/*
 * Set speed profile - CONSTANT_SPEED, LINEAR_SPEED (accelerated)
 * accel and decel are given in [full steps/s^2]
 */
void TimerStepperDriver::setSpeedProfile(Mode mode, short accel, short decel){
    profile.mode = mode;
    profile.accel = accel;
    profile.decel = decel;
}
void TimerStepperDriver::setSpeedProfile(struct Profile profile){
    this->profile = profile;
}

/*
 * Move the motor a given number of steps.
 * positive to move forward, negative to reverse
 */
void TimerStepperDriver::move(long steps){
    startMove(steps);
    while (nextAction());
}
/*
 * Move the motor a given number of degrees (1-360)
 */
void TimerStepperDriver::rotate(long deg){
    move(calcStepsForRotation(deg));
}
/*
 * Move the motor with sub-degree precision.
 * Note that using this function even once will add 1K to your program size
 * due to inclusion of float support.
 */
void TimerStepperDriver::rotate(double deg){
    move(calcStepsForRotation(deg));
}

/*
 * Set up a new move (calculate and save the parameters)
 */
void TimerStepperDriver::startMove(long steps, long time){
    float speed;
    // set up new move
    dir_state = (steps >= 0) ? HIGH : LOW;
    last_action_end = 0;
    steps_remaining = labs(steps);
    step_count = 0;
    rest = 0;
    switch (profile.mode){
    case LINEAR_SPEED:
        // speed is in [steps/s]
        speed = rpm * motor_steps / 60;
        if (time > 0){
            // Calculate a new speed to finish in the time requested
            float t = time / (1e+6);                  // convert to seconds
            float d = steps_remaining / microsteps;   // convert to full steps
            float a2 = 1.0 / profile.accel + 1.0 / profile.decel;
            float sqrt_candidate = t*t - 2 * a2 * d;  // in √b^2-4ac
            if (sqrt_candidate >= 0){
                speed = stepperMin(speed, (t - (float)sqrt(sqrt_candidate)) / a2);
            };
        }
        // how many microsteps from 0 to target speed
        steps_to_cruise = microsteps * (speed * speed / (2 * profile.accel));
        // how many microsteps are needed from cruise speed to a full stop
        steps_to_brake = steps_to_cruise * profile.accel / profile.decel;
        if (steps_remaining < steps_to_cruise + steps_to_brake){
            // cannot reach max speed, will need to brake early
            steps_to_cruise = steps_remaining * profile.decel / (profile.accel + profile.decel);
            steps_to_brake = steps_remaining - steps_to_cruise;
        }
        // Initial pulse (c0) including error correction factor 0.676 [us]
        step_pulse = (1e+6)*0.676*sqrt(2.0f/profile.accel/microsteps);
        // Save cruise timing since we will no longer have the calculated target speed later
        cruise_step_pulse = 1e+6 / speed / microsteps;
        break;

    case CONSTANT_SPEED:
    default:
        steps_to_cruise = 0;
        steps_to_brake = 0;
        step_pulse = cruise_step_pulse = STEP_PULSE(motor_steps, microsteps, rpm);
        if (time > steps_remaining * step_pulse){
            step_pulse = (float)time / steps_remaining;
        }
    }
}
/*
 * Alter a running move by adding/removing steps
 * FIXME: This is a naive implementation and it only works well in CRUISING state
 */
void TimerStepperDriver::alterMove(long steps){
    switch (getCurrentState()){
    case ACCELERATING: // this also works but will keep the original speed target
    case CRUISING:
        if (steps >= 0){
            steps_remaining += steps;
        } else {
            steps_remaining = stepperMax(steps_to_brake, steps_remaining+steps);
        };
        break;
    case DECELERATING:
        // would need to start accelerating again -- NOT IMPLEMENTED
        break;
    case STOPPED:
        startMove(steps);
        break;
    }
}
/*
 * Brake early.
 */
void TimerStepperDriver::startBrake(void){
    switch (getCurrentState()){
    case CRUISING:  // this applies to both CONSTANT_SPEED and LINEAR_SPEED modes
        steps_remaining = steps_to_brake;
        break;

    case ACCELERATING:
        steps_remaining = step_count * profile.accel / profile.decel;
        break;

    default:
        break; // nothing to do if already stopped or braking
    }
}
/*
 * Stop movement immediately and return remaining steps.
 */
long TimerStepperDriver::stop(void){
    long retval = steps_remaining;
    steps_remaining = 0;
    return retval;
}
/*
 * Return calculated time to complete the given move
 */
long TimerStepperDriver::getTimeForMove(long steps){
    float t;
    long cruise_steps;
    float speed;
    if (steps == 0){
        return 0;
    }
    switch (profile.mode){
        case LINEAR_SPEED:
            startMove(steps);
            cruise_steps = steps_remaining - steps_to_cruise - steps_to_brake;
            speed = rpm * motor_steps / 60;   // full steps/s
            t = (cruise_steps / (microsteps * speed)) +
                sqrt(2.0 * steps_to_cruise / profile.accel / microsteps) +
                sqrt(2.0 * steps_to_brake / profile.decel / microsteps);
            t *= (1e+6); // seconds -> micros
            break;
        case CONSTANT_SPEED:
        default:
            t = steps * STEP_PULSE(motor_steps, microsteps, rpm);
    }
    return round(t);
}
/*
 * Move the motor an integer number of degrees (360 = full rotation)
 * This has poor precision for small amounts, since step is usually 1.8deg
 */
void TimerStepperDriver::startRotate(long deg){
    startMove(calcStepsForRotation(deg));
}
/*
 * Move the motor with sub-degree precision.
 * Note that calling this function will increase program size substantially
 * due to inclusion of float support.
 */
void TimerStepperDriver::startRotate(double deg){
    startMove(calcStepsForRotation(deg));
}

/*
 * calculate the interval til the next pulse
 */
void TimerStepperDriver::calcStepPulse(void){
    if (steps_remaining <= 0){  // this should not happen, but avoids strange calculations
        return;
    }
    steps_remaining--;
    step_count++;

    if (profile.mode == LINEAR_SPEED){
        switch (getCurrentState()){
        case ACCELERATING:
            if (step_count < steps_to_cruise){
                step_pulse = step_pulse - (2*step_pulse+rest)/(4*step_count+1);
                rest = (2*step_pulse+rest) % (4*step_count+1);
            } else {
                // The series approximates target, set the final value to what it should be instead
                step_pulse = cruise_step_pulse;
                rest = 0;
            }
            break;

        case DECELERATING:
            step_pulse = step_pulse - (2*step_pulse+rest)/(-4*steps_remaining+1);
            rest = (2*step_pulse+rest) % (-4*steps_remaining+1);
            break;

        default:
            break; // no speed changes
        }
    }
}
/*
 * Yield to step control
 * Toggle step and return time until next change is needed (micros)
 */
void TimerStepperDriver::startAction(void){
    if (steps_remaining > 0){
        /*
         * DIR pin is sampled on rising STEP edge, so it is set first
         */
        digitalWrite(dir_pin, dir_state);
        digitalWrite(step_pin, HIGH);
		//next endAction
		timer.attach_us(1,&TimerStepperDriver::endAction);
    } else {
        // end of move
		timer.detach();
        last_action_end = 0;
        next_action_interval = 0;
    }
}

void TimerStepperDriver::endAction(void){
    if (steps_remaining > 0){
		unsigned m = micros();
        unsigned long pulse = step_pulse; // save value because calcStepPulse() will overwrite it
        calcStepPulse();
        digitalWrite(step_pin, LOW);
        // account for calcStepPulse() execution time; sets ceiling for max rpm on slower MCUs
        last_action_end = micros();
        m = last_action_end - m;
        next_action_interval = (pulse > m) ? pulse - m : 1;
		//next startAction
		timer.attach_us(next_action_interval,&TimerStepperDriver::startAction);
    } else {
        // end of move
		timer.detach();
        last_action_end = 0;
        next_action_interval = 0;
    }
}

long TimerStepperDriver::nextAction(void){
    if (steps_remaining > 0){
        delayMicros(next_action_interval, last_action_end);
        /*
         * DIR pin is sampled on rising STEP edge, so it is set first
         */
        digitalWrite(dir_pin, dir_state);
        digitalWrite(step_pin, HIGH);
        unsigned m = micros();
        unsigned long pulse = step_pulse; // save value because calcStepPulse() will overwrite it
        calcStepPulse();
        // We should pull HIGH for at least 1-2us (step_high_min)
        delayMicros(step_high_min);
        digitalWrite(step_pin, LOW);
        // account for calcStepPulse() execution time; sets ceiling for max rpm on slower MCUs
        last_action_end = micros();
        m = last_action_end - m;
        next_action_interval = (pulse > m) ? pulse - m : 1;
    } else {
        // end of move
        last_action_end = 0;
        next_action_interval = 0;
    }
    return next_action_interval;
}

enum TimerStepperDriver::State TimerStepperDriver::getCurrentState(void){
    enum State state;
    if (steps_remaining <= 0){
        state = STOPPED;
    } else {
        if (steps_remaining <= steps_to_brake){
            state = DECELERATING;
        } else if (step_count <= steps_to_cruise){
            state = ACCELERATING;
        } else {
            state = CRUISING;
        }
    }
    return state;
}
/*
 * Configure which logic state on ENABLE pin means active
 * when using SLEEP (default) this is active HIGH
 */
void TimerStepperDriver::setEnableActiveState(short state){
    enable_active_state = state;
}
/*
 * Enable/Disable the motor by setting a digital flag
 */
void TimerStepperDriver::enable(void){
    if IS_CONNECTED(enable_pin){
        digitalWrite(enable_pin, enable_active_state);
    };
    delayMicros(2);
}

void TimerStepperDriver::disable(void){
    if IS_CONNECTED(enable_pin){
        digitalWrite(enable_pin, (enable_active_state == HIGH) ? LOW : HIGH);
    }
}

short TimerStepperDriver::getMaxMicrostep(){
    return TimerStepperDriver::MAX_MICROSTEP;
}
