#pragma once

// Based on https://github.com/kaiaai/firmware/blob/iron/libraries/MotorController

#include <Arduino.h>
#include <PIDController.h>
#include <L298NX2.h>

#include "encoder.hpp"

#include "microros/microros_logger.hpp"

#define MAX_RPM 300

class MotorController
{
public:
	MotorController(L298N& motorDriver, volatile long& encoder_value, float PPR);

	bool setTargetRPM(float rpm);
	void update();
	float getShaftAngle();
	void reverseMotor(bool reversed);
	void setMaxRPM(float rpm);
	void setEncoderPPR(float ppr);
	void setPIDConfig(float kp, float ki, float kd);
	void setPIDKp(float kp);
	void setPIDKi(float ki);
	void setPIDKd(float kd);
	void setPIDPeriod(float period);
	void setPIDOnError(bool on_error);
	float getCurrentPWM();
	float getCurrentRPM();
	float getTargetRPM();
	float getMaxRPM();
	float getEncoderTPR();
	float getEncoderPPR();
	float getPIDKp();
	float getPIDKi();
	float getPIDKd();
	long int getEncoderValue();

private:
	L298N& motorDriver;

	volatile long &encoder;

	PIDController pidController;
	double Kp;
	double Ki;
	double Kd;

	void setPWM(float value);
	float pidPWM;
	float targetRPM;
	float measuredRPM;
	float pwm;
	float maxRPM;

	float encoderPPR;
	float encoderTPR;
	float encoderTPR_reciprocal;
	float ticksPerMicroSecToRPM;

	unsigned int pidUpdatePeriodUs;
	long int encPrev;
	bool setPointHasChanged;
	bool motorReversed;
	unsigned long tickSampleTimePrev;
};