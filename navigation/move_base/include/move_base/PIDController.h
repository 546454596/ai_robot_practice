// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	PIDController.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef __PIDController_H__
#define __PIDController_H__

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>               // for fabs()

using namespace std;
// Examples for _filter:
// f_cut = 10 Hz -> _alpha = 0.385869
// f_cut = 15 Hz -> _alpha = 0.485194
// f_cut = 20 Hz -> _alpha = 0.556864
// f_cut = 25 Hz -> _alpha = 0.611015
// f_cut = 30 Hz -> _alpha = 0.653373
#define PIDController_D_TERM_FILTER 0.556864f    // Default 100Hz Filter Rate with 20Hz Cutoff Frequency
#define PIDController_INTEGRAL_E 1
/// @class	PIDController
/// @brief	Object managing one PID control
class PIDController {
public:

    /// Constructor for PID that saves its settings to EEPROM
    ///
    /// @note	PIDs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_d       Initial value for the D term.
    /// @param  initial_imax    Initial value for the imax term.4
    ///
    PIDController(
        const float &   initial_p = 0.0,
        const float &   initial_i = 0.0,
        const float &   initial_d = 0.0,
        const float & initial_imax = 0.0):
        _integrator(0),
        _last_input(0),
        _last_derivative(0),
        _d_lpf_alpha(PIDController_D_TERM_FILTER)
    {
        _kp = initial_p;
        _ki = initial_i;
        _kd = initial_d;
        _imax = fabs(initial_imax);
        cout <<"PID: kP:" << _kp << " kI:" << _ki << " kD:" << _kd << " _imax:" << _imax << endl;
        // derivative is invalid on startup
        _last_derivative = NAN;
    }

    /// Iterate the PID, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    /// @param scaler	An arbitrary scale factor
    ///
    /// @returns		The updated control output.
    ///
    float       get_pid(float error, float dt);
    float       get_pd(float error, float dt);
    float       get_pi(float error, float dt);
    float       get_p(float error) const;
    float       get_i(float error, float dt);
    float       get_d(float error, float dt);

    /// Reset the PID integrator
    ///
    void        reset_I();

    /// Load gain properties
    ///
    void        load_gains(const char* filePath);

    /// Save gain properties
    ///
    void        save_gains(const char* filePath);

    /// Sets filter Alpha for D-term LPF
    void        set_d_lpf_alpha(int16_t cutoff_frequency, float time_step);

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator        () (const float    p,
                             const float    i,
                             const float    d,
                             const int16_t  imaxval) {
        _kp = p; _ki = i; _kd = d; _imax = abs(imaxval);
    }

    // accessors
    float       kP() const { return _kp; }
    float       kI() const { return _ki; }
    float       kD() const { return _kd; }
    float_t     imax() const { return _imax; }
    void        kP(const float v) { _kp=v; }
    void        kI(const float v) { _ki=v; }
    void        kD(const float v) { _kd=v; }
    void        imax(const int16_t v) { _imax=abs(v); }
    float       get_integrator() const { return _integrator; }
    void        set_integrator(float i) { _integrator = i; }


protected:
    float_t        _kp;
    float_t        _ki;
    float_t        _kd;
    float_t        _imax;

    float           _integrator;                                ///< integrator value
    float           _last_input;                                ///< last input for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter
    float           _d_lpf_alpha;                               ///< alpha used in D-term LPF
};

#endif // __PIDController_H__
