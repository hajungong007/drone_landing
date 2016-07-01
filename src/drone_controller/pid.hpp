#ifndef PID_HPP
#define PID_HPP
#include "ros/ros.h"
class PIDController{
public:
	/* coefficients for P, I, D*/
	float kP, kI, kD;
	/* coefficient for filtering the derivative values */
	float kN;
	float _P, _I, _D;
	//bool _bFirstFrame;
//protected:
	float old_err, cur_err;
	float old_time;
    //////////////////////////////
    float errs[20];
	/* error integration*/
	float cur_I;
	/* error derivative*/
	float old_D;
	PIDController():kP(0.0),kI(0.0),kD(0.0){reset();}
	//void loadParam(const char* filePath);
	void setParam(float _kP, float _kI, float _kD, float _kN);
	void reset();
	float getOutput(float curerr, float dt);
};
#endif 
