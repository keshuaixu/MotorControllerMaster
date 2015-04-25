#ifndef MotorControllerMaster_h
#define MotorControllerMaster_h
#include "Arduino.h"
#include "Wire.h"
#include "MiniMotorControllerDefinitions.h"

class MotorControllerMaster{
public:
	void begin();
	void calibrate(uint16_t wheelDistance, uint16_t wheelDiameter);
	//void setPID(float kp, float ki, float kd, float kvff);
	void setAcceleration(unsigned int forwardAcceleration, unsigned int ccwAcceleration, unsigned int forwardDeceleration, unsigned int ccwDeceleration);
	void goVelocity(int forwardVelocity, int ccwVelocity);	
	void brake();
	void coast();
	void getEncoder(long* left, long* right);
	byte isStandby();
	void heartbeat();
	void getGlobalPosition(long *x, long *y);
};

#endif