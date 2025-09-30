// pid_controller.c
#include "pid_controller.h"
#include <math.h> // for fmaxf/fminf (or use ternary if no libm)

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd,
              float output_min, float output_max,
              float integral_min, float integral_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->output = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->first_pass = 1;
}

float PID_Compute(PID_Controller* pid, float measurement, float dt) {
    if (dt <= 0.0f) return pid->output; // Avoid division by zero or negative time

    float error = pid->setpoint - measurement;

    // Proportional term
    float proportional = pid->Kp * error;

    // Integral term with anti-windup
    pid->integral += pid->Ki * error * dt;
    // Clamp integral to prevent windup
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    // Derivative on measurement (avoids derivative kick on setpoint change)
    float derivative = 0.0f;
    if (!pid->first_pass) {
        derivative = -(pid->Kd * (measurement - pid->prev_measurement) / dt);
    } else {
        pid->first_pass = 0;
    }

    // Compute total output
    float output = proportional + pid->integral + derivative;

    // Clamp final output
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    // Save state
    pid->prev_measurement = measurement;
    pid->output = output;

    return output;
}
