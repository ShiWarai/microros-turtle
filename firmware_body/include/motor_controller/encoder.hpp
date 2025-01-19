#pragma once

#include <Arduino.h>

#define DECLARE_ENCODER_WITH_NAME(NAME, ENCODER_PIN_A, ENCODER_PIN_B)\
volatile long encoder_##NAME = 0;\
void updateEncoder##NAME()\
{\
	if (digitalRead(ENCODER_PIN_A) > digitalRead(ENCODER_PIN_B))\
		encoder_##NAME++;\
	else\
		encoder_##NAME--;\
};

#define INIT_ENCODER_WITH_NAME(NAME, ENCODER_PIN_A, ENCODER_PIN_B)\
pinMode(ENCODER_PIN_A, INPUT);\
pinMode(ENCODER_PIN_B, INPUT);\
attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder##NAME, RISING);