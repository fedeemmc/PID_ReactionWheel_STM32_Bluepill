#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* Controller parameters */
#define PID_KP  0.0005
#define PID_KI  0.0001
#define PID_KD  0.000025
#define SAMPLE_TIME_S 0.004
#define MAX_SPEED 4000
#define MIN_SPEED 1
#define MAX_PID 40
#define MIN_PID -40

typedef struct{
double kp, ki, kd, lastErr, cumulErr, out;

}PIDController;

//void PIDController_Init(PIDController *pid);
double calcError(double setPoint, double current);
double compute(PIDController *pid, double setPoint,double current, long detaT);
void reset(PIDController *pid);
#endif
