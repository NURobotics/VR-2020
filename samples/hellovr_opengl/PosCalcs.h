#include <openvr.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "InverseK.h"
#include "SerialClass.h"
using namespace vr;

enum HumanName {
	Albert,
	Paulina,
	Sam,
	John
};

struct ViveDevice {
	enum DeviceName {
		HMD,
		Right,
		Left,

		Wrong
	};

	DeviceName name;

	HmdVector3_t XYZ;
	HmdQuaternion_t ROT;

	bool valid;
	ETrackingResult trackingResult;

	ViveDevice(char name);
	void setPosAndRot(HmdVector3_t p, HmdQuaternion_t r);

	char* getEnglishPoseValidity();
	char* getEnglishTrackingResultForPose();
	char* print();
};

struct ViveController : ViveDevice {
	float dPadX;
	float dPadY;
	float trigger;

	HmdVector3_t relativeXYZ;
	HmdQuaternion_t relativeROT;

	bool calibarated = false;

	void setInputs(float dx, float dy, float t);
	char* print(bool rel);

	ViveController(char name) : ViveDevice(name) {
		dPadX = 0;
		dPadY = 0;
		trigger = 0;
	}
};

struct Human {
	float armLength;
	float headToArmAcross;

	HmdVector3_t shoulderPos;
	HmdVector3_t headShoulderDistance;

	void calcHeadShoulderOffset(ViveController con, ViveDevice H);

	Human(HumanName n);
};

struct AllViveDevices {
	ViveDevice H = ViveDevice('H');
	ViveController R = ViveController('R');
	ViveController L = ViveController('L');

	void calcRelHandPos(Human Hu);
	void print(bool rel);
};

struct RobotArm {
	float upperArmLength = 0.096;
	float foreArmLength = 0.0906;
	float handLength = 0.096;

	Link base, upperarm, forearm, hand;

	HmdVector3_t handPosition;

	float human2ArmConversion;

	float shoulderAngle;
	float upperArmAngle;
	float foreArmAngle;
	float handAngle = -90;
	int gripAngle;

	Serial* SP;

	RobotArm(float humanArmLength, const char* serial_port);
	virtual ~RobotArm();
	void calcHandPosition(ViveController R);
	void calcAngles(ViveController R);

	bool send(int base, int shoulder, int elbow, int wrist, int grip);
};