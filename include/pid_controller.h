// pid_controller.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
    float Kp;          // Proportional gain
    float Ki;          // Integral gain
    float Kd;          // Derivative gain

    float setpoint;    // Desired value
    float output;      // Controller output
    float integral;    // Integral term
    float prev_error;  // Previous error (for derivative)
    float prev_measurement; // Previous process variable (for derivative on measurement)

    float output_min;  // Output saturation limits
    float output_max;

    float integral_min; // Integral anti-windup limits
    float integral_max;

    int first_pass;    // Flag to skip derivative on first call
} PID_Controller;

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd,
              float output_min, float output_max,
              float integral_min, float integral_max);

float PID_Compute(PID_Controller* pid, float measurement, float dt);

#endif
