#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "../include/lmpc_pid.hpp"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki , double Klim);
        ~PIDImpl();
        double calculate( double setpoint, double pv );
        void setPIDParams(double dt, double Kp, double Ki, double Kd, double Klim);
        double getPout();
        double getIout();
        double getDout();

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _Klim;
        double _pre_error;
        double _integral;
        double _Pout;
        double _Iout;
        double _Dout;
};



PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki, double Klim )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki,Klim);
}
void PID::setPIDParams(double dt, double Kp, double Ki, double Kd, double Klim){
    pimpl->setPIDParams(dt,Kp,Ki,Kd,Klim);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
double PID::getPout(){ return pimpl->getPout();}
double PID::getIout(){ return pimpl->getIout();}
double PID::getDout(){ return pimpl->getDout();}
PID::~PID()
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki, double Klim) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _Klim(Klim),
    _pre_error(0),
    _integral(0),
    _Pout(0),
    _Iout(0),
    _Dout(0)
{
}

// SETTERS

void PIDImpl::setPIDParams(double dt, double Kp, double Ki, double Kd, double Klim){
    _dt = dt;
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _Klim = Klim;
}

// GETTERS

double PIDImpl::getPout(){return _Pout;}
double PIDImpl::getIout(){return _Iout;}
double PIDImpl::getDout(){return _Dout;}

// FUNCTIONS

double PIDImpl::calculate( double setpoint, double pv )
{

    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // std::cout << "P = " << Pout << std::endl;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // std::cout << "I = " << Iout << std::endl;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // std::cout << "P + I = " << output << std::endl;

    // Restrict to max/min
    if( output > _max ){
        if(_integral > 0) _integral -= _Klim*error*_dt;   
        output = _max;
    }else if( output < _min ){
        if(_integral < 0 ) _integral -= _Klim*error*_dt;
        output = _min;
    }    

    // std::cout << "saturated (P + I) = " << output << std::endl ;   

    // Save error to previous error
    _pre_error = error;

    //Save P I D values
    _Pout = Pout;
    _Iout = Iout;
    _Dout = Dout;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
