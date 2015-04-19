#ifndef MotorControllerMaster_h
#define MotorControllerMaster_h
#include "Arduino.h"
#include "Wire.h"
#include "MiniMotorControllerDefinitions.h"

class MotorControllerMaster{
public:
	void begin();
	void setAcceleration(unsigned int forwardAcceleration, unsigned int ccwAcceleration, unsigned int forwardDeceleration, unsigned int ccwDeceleration);
	void goVelocity(int forwardVelocity, int ccwVelocity);	
	void brake();
	void coast();
	void getEncoder(long* left, long* right);
	byte isStandby();
	void heartbeat();
};

#endif