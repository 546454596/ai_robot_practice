// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	PIDController.cpp
/// @brief	Generic PID algorithm

#include "move_base/PIDController.h"


float PIDController::get_p(float error) const
{
    return (float)error * _kp;
}

float PIDController::get_i(float error, float dt)
{
    if((_ki != 0) && (dt != 0)) {
        if(fabs(error)<PIDController_INTEGRAL_E){
        _integrator += ((float)error * _ki) * dt;
        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
        return _integrator;
        }
        else
            {
            return 0;
        }

    }
    return 0;
}

float PIDController::get_d(float input, float dt)
{
    if ((_kd != 0) && (dt != 0)) {
        float derivative;
        if (isnan(_last_derivative)) {
            // we've just done a reset, suppress the first derivative
            // term as we don't want a sudden change in input to cause
            // a large D output change
            derivative = 0;
            _last_derivative = 0;
        } else {
            // calculate instantaneous derivative
            derivative = (input - _last_input) / dt;
        }

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        derivative = _last_derivative + _d_lpf_alpha * (derivative - _last_derivative);

        // update state
        _last_input             = input;
        _last_derivative    = derivative;


        // add in derivative component
        return _kd * derivative;
    }
    return 0;
}

float PIDController::get_pi(float error, float dt)
{
    return get_p(error) + get_i(error, dt);
}

float PIDController::get_pd(float error, float dt)
{
    return get_p(error) + get_d(error, dt);
}

float PIDController::get_pid(float error, float dt)
{
    return get_p(error) + get_i(error, dt) + get_d(error, dt);
}

void PIDController::reset_I()
{
    _integrator = 0;
    // mark derivative as invalid
    _last_derivative = NAN;
}

void PIDController::load_gains(const char* filePath)
{
    std::ifstream file(filePath);
    file >> _kp >> _ki >> _kd >> _imax;
    cout <<" kP:" << _kp << " kI:" << _ki << " kD:" << _kd << " _imax:" << _imax << endl;
     _imax = fabs(_imax);
    file.close();
}

void PIDController::save_gains(const char* filePath)
{
    std::ofstream file(filePath);
    file << _kp <<" "<< _ki<<" " << _kd <<" "<< _imax;
     file.close();
}

void PIDController::set_d_lpf_alpha(int16_t cutoff_frequency, float time_step)
{
    // calculate alpha
    float rc = 1/(2*M_PI*cutoff_frequency);
    _d_lpf_alpha = time_step / (time_step + rc);
}
