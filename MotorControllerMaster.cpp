#include "MotorControllerMaster.h"

void MotorControllerMaster::begin(){
}

void MotorControllerMaster::setAcceleration(unsigned int forwardAcceleration, unsigned int ccwAcceleration, unsigned int forwardDeceleration, unsigned int ccwDeceleration){
	byte send[9];
	send[0] = COMMAND_SETACCELERATION;
	send[1] = forwardAcceleration;
	send[2] = forwardAcceleration >> 8;
	send[3] = ccwAcceleration;
	send[4] = ccwAcceleration >> 8;
	send[5] = forwardDeceleration;
	send[6] = forwardDeceleration >> 8;
	send[7] = ccwDeceleration;
	send[8] = ccwDeceleration >> 8;
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(send,9);
	int e = Wire.endTransmission();
	if (e != 0){
		Serial.println("i2c error");
		return;
	}
}

void MotorControllerMaster::calibrate(uint16_t wheelDistance, uint16_t wheelDiameter){
	byte send[5];
	send[0] = COMMAND_CALIBRATE;
	send[1] = wheelDistance;
	send[2] = wheelDistance >> 8;
	send[3] = wheelDiameter;
	send[4] = wheelDiameter >> 8;
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(send,5);
	int e = Wire.endTransmission();
	if (e != 0){
		Serial.println("i2c error");
		return;
	}
}

void MotorControllerMaster::getGlobalPosition(long *x, long *y){
	int32_t remoteleft;
	int32_t remoteright;
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(COMMAND_GETGLOBALPOSITION);
	int e = Wire.endTransmission();
	if (e != 0){
		Serial.println("i2c error");
		return;
	}
	delayMicroseconds(17);
	int length = Wire.requestFrom(MOTOR_CONTROLLER_ADDRESS, 8);
	if (length != 8){
		Serial.println("i2c error2");
	}
	remoteleft = (int32_t) Wire.read();
	remoteleft |= (int32_t) Wire.read() << 8UL;
	remoteleft |= (int32_t) Wire.read() << 16UL;
	remoteleft |= (int32_t) Wire.read() << 24UL;
	remoteright = (int32_t) Wire.read();
	remoteright |= (int32_t) Wire.read() << 8UL;
	remoteright |= (int32_t) Wire.read() << 16UL;
	remoteright |= (int32_t) Wire.read() << 24UL;	
	*x = remoteleft;
	*y = remoteright;	
}
/*
void MotorControllerMaster::setPID(float kp, float ki, float kd, float kvff){
	byte send[17];
	send[0] = COMMAND_PID;
	send[1] = kp;
	send[2] = kp >> 8UL;
	send[3] = kp >> 16UL;
	send[4] = kp >> 24UL;
	send[5] = ki;
	send[6] = ki >> 8UL;
	send[7] = ki >> 16UL;
	send[8] = ki >> 24UL;
	send[9] = kd;
	send[10] = kd >> 8UL;
	send[11] = kd >> 16UL;
	send[12] = kd >> 24UL;
	send[13] = kvff;
	send[14] = kvff >> 8UL;
	send[15] = kvff >> 16UL;
	send[16] = kvff >> 24UL;
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(send,17);
	int e = Wire.endTransmission();
	if (e != 0){
		Serial.println("i2c error");
		return;
	}
}
*/
void MotorControllerMaster::goVelocity(int forwardVelocity, int ccwVelocity){
	byte send[5];
	send[0] = COMMAND_GOVELOCITY;
	send[1] = forwardVelocity;
	send[2] = forwardVelocity >> 8;
	send[3] = ccwVelocity;
	send[4] = ccwVelocity >> 8;
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(send,5);
	int e = Wire.endTransmission();
	if (e != 0){
		Serial.println("i2c error");
		return;
	}
}

void MotorControllerMaster::brake(){
	byte send[1];
	send[0] = COMMAND_BRAKE;
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(send,1);
	int e = Wire.endTransmission();
	if (e != 0){
		Serial.println("i2c error");
		return;
	}
}

void MotorControllerMaster::coast(){
	byte send[1];
	send[0] = COMMAND_COAST;
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(send,1);
	int e = Wire.endTransmission();
	if (e != 0){
		Serial.println("i2c error");
		return;
	}
}

void MotorControllerMaster::getEncoder(long* left, long* right){
	int32_t remoteleft;
	int32_t remoteright;
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(COMMAND_REPORTENCODER);
	int e = Wire.endTransmission();
	if (e != 0){
		Serial.println("i2c error");
		return;
	}
	delayMicroseconds(17);
	int length = Wire.requestFrom(MOTOR_CONTROLLER_ADDRESS, 8);
	if (length != 8){
		Serial.println("i2c error2");
	}
	remoteleft = (int32_t) Wire.read();
	remoteleft |= (int32_t) Wire.read() << 8UL;
	remoteleft |= (int32_t) Wire.read() << 16UL;
	remoteleft |= (int32_t) Wire.read() << 24UL;
	remoteright = (int32_t) Wire.read();
	remoteright |= (int32_t) Wire.read() << 8UL;
	remoteright |= (int32_t) Wire.read() << 16UL;
	remoteright |= (int32_t) Wire.read() << 24UL;	
	*left = remoteleft;
	*right = remoteright;
}

byte MotorControllerMaster::isStandby(){
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(COMMAND_REPORTSTANDBY);
	int e = Wire.endTransmission();	
	if (e != 0){
		Serial.println("i2c error");
		return -2;
	}
	delayMicroseconds(17);
	int length = Wire.requestFrom(MOTOR_CONTROLLER_ADDRESS, 1);
	if (length != 1){
		Serial.println("i2c error2");
	}
	int8_t isStandby = Wire.read();
	return isStandby;
}

void MotorControllerMaster::heartbeat(){
	Wire.beginTransmission(MOTOR_CONTROLLER_ADDRESS);
	Wire.write(COMMAND_HEARTBEAT);
	Wire.endTransmission();		
}