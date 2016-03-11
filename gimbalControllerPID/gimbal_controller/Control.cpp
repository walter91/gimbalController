#include <Arduino.h>
#include "Control.h"
#include <math.h>

Control::Control(void)
{
	
	
	
}

void Control::begin()
{
	
	
	
}

void Control::set_time_filter(float Ts, float alpha, float tau)
{
	_Ts = Ts;
	_alpha = alpha;
	_tau = tau;
}

void Control::set_gains(float kp, float ki, float kd)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;	
	
}

void Control::set_precision(int pwmBits, int adcBits)
{
	_pwmBits = pwmBits;
	_adcBits = adcBits;
	
	//ADC and PWM precision
	analogWriteResolution(pwmBits);
	analogReadResolution(adcBits);
	
}

void Control::set_antiwindup(float intThreshHigh, float intThreshLow)
{
	_intThreshHigh = intThreshHigh;
	_intThreshLow = intThreshLow;
}

float Control::pid(float state, float command, bool flag)
{
	
	if(flag)
	{
		_integrator = 0.0;
		_differentiator = 0.0;
		_error_d1 = 0.0;
	}
	
	float error = command - state;
	//Serial.println(error);
	
	if(abs(error) < _intThreshHigh && abs(error) > _intThreshLow)
	{
		_integrator = _integrator + (_Ts/2.0)*(error + _error_d1);
	}
	else
	{
		_integrator = 0.0;
	}
	
	_differentiator = ((2.0*_tau - _Ts)/(2.0*_tau + _Ts))*_differentiator + (2.0/(2.0*_tau + _Ts))*(error - _error_d1);
	
	_error_d1 = error;
	
	//Serial.print(_kp);Serial.print("\t");Serial.print(_ki);Serial.print("\t");Serial.println(_kd);
	
	//Serial.println(saturate(_kp*error + _ki*_integrator + _kd*_differentiator, _contThresh, 0.0));
	
	//return(_kp*error + _ki*_integrator + _kd*_differentiator);
	return(saturate((_kp*error + _ki*_integrator + _kd*_differentiator), 1.0, -1.0));

}

void Control::set_pins(int pwmPin, int dirPin)
{
	_pwmPin = pwmPin;
	_dirPin = dirPin;
	
	pinMode(_pwmPin, OUTPUT);
	pinMode(_dirPin, OUTPUT);
}


float Control::saturate(float input, float highLimit, float lowLimit)
{
	float output;
	
	if(input >= highLimit)
	{
		output = highLimit;
		//Serial.println("High Limit");
	}
	else if(input < lowLimit)
	{
		output = lowLimit;
		//Serial.println("Low Limit");
	}
	else
	{
		output = input;
		//Serial.println("Good");
	}
	
	//Serial.println(output);
	return(output);
}