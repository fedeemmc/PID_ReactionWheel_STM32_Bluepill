#include <PIDController.h>


double calcError(double setPoint, double current){
    return setPoint - current;
  }

double compute(PIDController *pid, double setPoint,double current, long detaT){
     /*Compute all the working error variables*/
     double error = calcError(setPoint, current);
     double dErr = (detaT==0)?0:calcError(error, pid->lastErr) / detaT;

     /*Remember some variables for next time*/
     pid->lastErr = error;
     pid->cumulErr += error * detaT;

     /*Compute PID Output*/
     pid->out  = pid->kp * error + pid->ki * pid->cumulErr +  pid->kd * dErr;
 	 if(pid->out > MAX_PID) pid->out   = MAX_PID;
 	 else if (pid->out  < MIN_PID)  pid->out = MIN_PID;
     return pid->out;
  }
void reset(PIDController *pid){

	pid->lastErr=0; pid->cumulErr=0;pid->out=0;
  }
