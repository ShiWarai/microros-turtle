#include "motor_controller/motor_controller.hpp"

MotorController::MotorController(L298N &motorDriver, volatile long &encoder_value, float PPR)
	: encoder(encoder_value), motorDriver(motorDriver)
{
	pidController = GyverPID(0.0, 0.0, 0.0);
	pidController.setLimits(-255, 255);
	pidController.setDirection(0);
	pidController.setMode(0);

	setEncoderPPR(PPR);
	
	setMaxRPM(MAX_RPM);
}

void MotorController::update(float dt, float correction)
{
	if (targetIsChanged) {
		straight_Kp = 0;
		correction = 0;

		targetIsChanged = false;
	} else {
		straight_Kp = max(-10.0f, min(straight_Kp, 10.0f));
		correction = max(-5.0f, min(correction, 5.0f));
	}

	straight_Kp += correction;

	long int encNow = getEncoderValue();
	long int encDelta = encNow - encPrev;
	encPrev = encNow;

	measuredRPM = ((float)encDelta) / ((float)dt) * ticksPerMillisecToRPM;

	pidController.setpoint = targetRPM;
	pidController.input = measuredRPM;
	pidController.setDt(dt);

	pidtype pid = pidController.getResult();
	pidPWM = pid + Kff * targetRPM + straight_Kp;

	setPWM(pidPWM);

	// char str[128];
	// sprintf(str, "C: %.2f, T: %.2f, PID: %.2f, pidPWM: %.2f, K_p: %.2f, Kff: %.2f", measuredRPM, targetRPM, pid, pidPWM, straight_Kp, Kff);
	// MicroROSLogger::log(str, "update()", "motor_controller.cpp", LogLevel::INFO, false);
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
	ticksPerMillisecToRPM = 1e3 * 60.0 / ppr;
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

void MotorController::setPIDConfig(float kp, float ki, float kd, float kff)
{
	Kp = kp;
	Ki = ki;
	Kd = kd;
	Kff = kff;
	
	pidController.Kp = kp;
	pidController.Ki = ki;
	pidController.Kd = kd;
}

void MotorController::setPIDKp(float kp)
{
	setPIDConfig(kp, getPIDKi(), getPIDKd(), getPIDKff());
}

void MotorController::setPIDKi(float ki)
{
	setPIDConfig(getPIDKp(), ki, getPIDKd(), getPIDKff());
}

void MotorController::setPIDKd(float kd)
{
	setPIDConfig(getPIDKp(), getPIDKi(), Kd, getPIDKff());
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

float MotorController::getPIDKff()
{
	return Kff;
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

float SpeedMatchingRegulator::compute(float leftSpeed, float rightSpeed, float leftTargetSpeed, float rightTargetSpeed) {
	float koef;

	if(rightTargetSpeed == 0 && leftTargetSpeed == 0)
		return 0;
	else if(rightTargetSpeed == 0)
		return 0;
	else if(leftTargetSpeed == 0)
		return 0;
	else
		koef = leftTargetSpeed / rightTargetSpeed;

	return kp_ * (leftSpeed - rightSpeed * koef);
}

float SpeedMatchingRegulator::getKp() {
	return kp_;
}

void SpeedMatchingRegulator::setKp(float Kp) {
	kp_ = Kp;
}