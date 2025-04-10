#ifndef _PID_H_
#define _PID_H_

#include "ros/ros.h"

class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        PID( double dt, double Kp, double Kd, double Ki, double Klim);
        
        //Changes the PID gains
        void setPIDParams(double dt, double Kp, double Ki, double Kd, double Klim);
        double getPout();
        double getIout();
        double getDout();
        
        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv);
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif