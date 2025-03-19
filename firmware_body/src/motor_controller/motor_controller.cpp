#include "motor_controller/motor_controller.hpp"

MotorController::MotorController(L298N &motorDriver, volatile long &encoder_value, float PPR)
	: encoder(encoder_value), motorDriver(motorDriver)
{
	pidController = GyverPID(0.0, 0.0, 0.0);
	pidController.setLimits(-255, 255);
	setEncoderPPR(PPR);
	
	setMaxRPM(MAX_RPM);
}

void MotorController::update(int64_t dt, float correction)
{
	if (targetIsChanged) {
		straight_Kp = 0;
		correction = 0;

		targetIsChanged = false;
	} else {
		straight_Kp = max(-10.0f, min(straight_Kp, 10.0f));
	}

	long int encNow = getEncoderValue();
	long int encDelta = encNow - encPrev;
	encPrev = encNow;

	measuredRPM = ((float)encDelta) / ((float)dt) * ticksPerMicroSecToRPM;

	straight_Kp += correction;

	pidController.setpoint = targetRPM + straight_Kp;
	pidController.input = measuredRPM;
	pidController.setDirection(0);
	pidController.setDt(dt);

	pidPWM = pidController.getResult();

	setPWM(pidPWM);

	// char str[128];
	// sprintf(str, "C: %.2f, %.2f, %.2f, %.2f", measuredRPM, targetRPM, pidPWM, straight_Kp);
	// MicroROSLogger::log(str, "update()", "motor_controller.cpp", LogLevel::INFO, true);
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

	if(fabs(value) >= 50.0) {
		if ((value > 0 && !motorReversed) || (value < 0 && motorReversed))
			motorDriver.forward();
		else
			motorDriver.backward();
	} else {
		motorDriver.stop();
	}

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
	
	pidController.Kp = kp;
	pidController.Ki = ki;
	pidController.Kd = kd;
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
		targetIsChanged = true;
		
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