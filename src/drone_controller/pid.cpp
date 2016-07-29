
#include "pid.hpp"


using namespace std;


void PIDController::setParam(float _kP, float _kI, float _kD, float _kN){
	kP = _kP;
	kI = _kI;
	kD = _kD;
	kN = _kN;
}

void PIDController::reset(){
	old_err = 0;
	cur_err = 0;
	old_D = 0;
	cur_I = 0;
	old_D = 0;
	//_bFirstFrame = true;
}

float PIDController::getOutput(float curerr, float t){
	old_err = cur_err;
	cur_err = curerr;
	for (int i =0; i<10; ++i) errs[i+1]=errs[i];
        errs[0]=curerr;
    
	float s = 0;
        float dt=t-old_time;
	cur_I += cur_err*dt;
	_P = cur_err;
	_I = cur_I;
	float Derr = (cur_err - old_err)/dt;

        float aveerr = (errs[0]+errs[1])/2;
        float pasterr =0;
        for (int i=1; i<=3; ++i) pasterr+=errs[i];
        pasterr/=3;
        _D = aveerr - pasterr;
	old_time=t;
	return kP*_P + kI*_I + kD*_D;
}
