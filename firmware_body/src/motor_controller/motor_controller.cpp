#include "motor_controller/motor_controller.hpp"

MotorController::MotorController(L298N &motorDriver, volatile long &encoder_value, float PPR)
	: encoder(encoder_value), motorDriver(motorDriver)
{
	// Kp = 20;
	// Ki = 0.1;
	// Kd = 200;

	pidController.begin();
	pidController.limit(-255, 255);
	setEncoderPPR(PPR);
	
	setMaxRPM(MAX_RPM);

	targetRPM = 0;
	measuredRPM = 0;
	pidPWM = 0;
	encPrev = 0;
	tickSampleTimePrev = 0;
	motorReversed = false;

	float pidPeriod = 1.0;
	pidUpdatePeriodUs = (unsigned int)round(pidPeriod * 1e6);

	pwm = 0;
}

void MotorController::update(float dt, float correction)
{
	// if ((dt < pidUpdatePeriodUs))
	// 	return;

	long int encNow = getEncoderValue();
	long int encDelta = encNow - encPrev;
	encPrev = encNow;

	measuredRPM = ((float)encDelta) / ((float)dt) * ticksPerMicroSecToRPM;

	targetRPM += correction;
	pidController.setpoint(targetRPM );
	pidPWM = pidController.compute(measuredRPM);

	setPWM(pidPWM);

	char str[128];
	sprintf(str, "C: %.2f, %.2f, %.2f, %.2f", measuredRPM, targetRPM, pidPWM, correction);
	MicroROSLogger::log(str, "update()", "motor_controller.cpp", LogLevel::INFO, true);
}

void MotorController::setPWM(float value)
{
	if (value == pwm)
		return;

	if (value > 255)
		value = 255;
	else if (value < -255)
		value = -255;

	motorDriver.setSpeed(abs(value));

	if ((value > 0 && !motorReversed) || (value < 0 && motorReversed))
		motorDriver.forward();
	else
		motorDriver.backward();

	pwm = value;
}

float MotorController::getShaftAngle()
{
	return TWO_PI * getEncoderValue() * encoderTPR_reciprocal;
}

void MotorController::setMaxRPM(float rpm)
{
	maxRPM = abs(rpm);
}

void MotorController::setEncoderPPR(float ppr)
{
	if (ppr <= 0)
		return;

	encoderPPR = ppr;
	ticksPerMicroSecToRPM = 1e6 * 60.0 / ppr;
}

float MotorController::getEncoderPPR()
{
	return encoderPPR;
}

float MotorController::getEncoderTPR()
{
	return encoderTPR;
}

float MotorController::getMaxRPM()
{
	return maxRPM;
}

float MotorController::getCurrentPWM()
{
	return motorReversed ? -pwm : pwm;
}

float MotorController::getCurrentRPM()
{
	return measuredRPM;
}

float MotorController::getTargetRPM()
{
	return targetRPM;
}

void MotorController::setPIDConfig(float kp, float ki, float kd)
{
	Kp = kp;
	Ki = ki;
	Kd = kd;
	
	pidController.tune(Kp, Ki, Kd); // настраиваем PID аргументы kP, kI, kD
}

void MotorController::setPIDKp(float kp)
{
	setPIDConfig(kp, getPIDKi(), getPIDKd());
}

void MotorController::setPIDKi(float ki)
{
	setPIDConfig(getPIDKp(), ki, getPIDKd());
}

void MotorController::setPIDKd(float kd)
{
	setPIDConfig(getPIDKp(), getPIDKi(), kd);
}

float MotorController::getPIDKp()
{
	return Kp;
}

float MotorController::getPIDKi()
{
	return Ki;
}

float MotorController::getPIDKd()
{
	return Kd;
}

bool MotorController::setTargetRPM(float rpm)
{
	if (targetRPM == rpm)
		return false;

	if(abs(rpm) <= maxRPM) {
		targetRPM = rpm;
		pidController.setpoint(targetRPM);
		
		return true;
	}
	else {
		return false;
	}
}

void MotorController::reverseMotor(bool reversed)
{
	motorReversed = reversed;
}

long int MotorController::getEncoderValue()
{
	return encoder;
}