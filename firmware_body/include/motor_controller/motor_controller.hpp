#pragma once

#define PID_OPTIMIZED_I

#include <Arduino.h>
#include <GyverPID.h>
#include <L298NX2.h>

#include "encoder.hpp"

#include "microros/microros_logger.hpp"

#define MAX_RPM 300

class SpeedMatchingController {
	public:
		SpeedMatchingController(float kp) : kp_(kp) {}

		float compute(float leftSpeed, float rightSpeed, float leftTargetSpeed, float rightTargetSpeed) {
			float koef;

			if(rightTargetSpeed == 0)
				koef = (leftTargetSpeed == 0) ? 0 : INFINITY;
			else
				koef = leftTargetSpeed / rightTargetSpeed;

			return kp_ * (leftSpeed - rightSpeed * koef);
		}

	private:
		float kp_;
};

class MotorController
{
public:
	MotorController(L298N& motorDriver, volatile long& encoder_value, float PPR);

	bool setTargetRPM(float rpm);
	void update(int64_t dt, float correction = 0);
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
	void setPWM(float value);
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

	GyverPID pidController;
	double Kp = 0;
	double Ki = 0;
	double Kd = 0;
	float straight_Kp = 0;

	float pidPWM = 0;
	float targetRPM = 0;
	float measuredRPM = 0;
	float pwm = 0;
	float maxRPM = 0;
	bool targetIsChanged = false;

	float encoderPPR;
	float encoderTPR;
	float encoderTPR_reciprocal;
	float ticksPerMicroSecToRPM;

	long int encPrev = 0;
	bool motorReversed = false;
	unsigned long tickSampleTimePrev = 0;
};