#include "PosCalcs.h"

void ViveDevice::setPosAndRot(HmdVector3_t p, HmdQuaternion_t r) {
	XYZ = p;
	ROT = r;
}

char* ViveDevice::getEnglishPoseValidity() {
	char* buf = new char[50];
	if (valid)
		sprintf(buf, "Valid");
	else
		sprintf(buf, "Invalid");
	return buf;
}

char* ViveDevice::getEnglishTrackingResultForPose()
{
	char* buf = new char[50];
	switch (trackingResult)
	{
	case vr::ETrackingResult::TrackingResult_Uninitialized:
		sprintf(buf, "Invalid tracking result");
		break;
	case vr::ETrackingResult::TrackingResult_Calibrating_InProgress:
		sprintf(buf, "Calibrating in progress");
		break;
	case vr::ETrackingResult::TrackingResult_Calibrating_OutOfRange:
		sprintf(buf, "Calibrating Out of range");
		break;
	case vr::ETrackingResult::TrackingResult_Running_OK:
		sprintf(buf, "Running OK");
		break;
	case vr::ETrackingResult::TrackingResult_Running_OutOfRange:
		sprintf(buf, "WARNING: Running Out of Range");
		break;
	default:
		sprintf(buf, "Default");
		break;
	}
	return buf;
}

char* ViveDevice::print() {
	HmdVector3_t pos = XYZ;
	char* cB = new char[50];
	if (valid)
		sprintf(cB, "x:%.3f y:%.3f z:%.3f", pos.v[0], pos.v[1], pos.v[2]);
	else
		sprintf(cB, "            INVALID");
	return cB;
}

ViveDevice::ViveDevice(char name) {
	switch (name) {
		case 'H': {
			name = HMD;
		}
		case 'L': {
			name = Left;
		}
		case 'R': {
			name = Right;
		}
		default: {
			name = Wrong;
		}
	}
}

void ViveController::setInputs(float dx, float dy, float t) {
	dPadX = dx;
	dPadX = dy;
	trigger = t;
}

char* ViveController::print(bool rel) {
	HmdVector3_t pos = rel ? relativeXYZ : XYZ;
	char* cB = new char[50];
	if (valid)
		sprintf(cB, "x:%.3f y:%.3f z:%.3f", pos.v[0], pos.v[1], pos.v[2]);
	else
		sprintf(cB, "            INVALID");
	return cB;
}

void AllViveDevices::calcRelHandPos(Human Hu) {
	for (int i = 0; i < 3; i++) {
		R.relativeXYZ.v[i] = R.XYZ.v[i] - H.XYZ.v[i] - Hu.headShoulderDistance.v[i];
		if (i == 0)
			L.relativeXYZ.v[i] = L.XYZ.v[i] - H.XYZ.v[i] + Hu.headShoulderDistance.v[i];
		else
			L.relativeXYZ.v[i] = L.XYZ.v[i] - H.XYZ.v[i] - Hu.headShoulderDistance.v[i];
	}
}

void AllViveDevices::print(bool rel) {
	char* coordsBuf = new char[1024];
	char* trackBuf = new char[1024];
	char* rotBuf = new char[1024];

	char* bufL = new char[100];
	char* bufR = new char[100];

	sprintf(coordsBuf, "HMD %-28.28s", H.print());
	sprintf(trackBuf, "HMD: %-25.25s %-7.7s ", H.getEnglishTrackingResultForPose(), H.getEnglishPoseValidity());
	sprintf(rotBuf, "HMD: qw:%.2f qx:%.2f qy:%.2f qz:%.2f", H.ROT.w, H.ROT.x, H.ROT.y, H.ROT.z);

	sprintf(coordsBuf, "%s %s: %-28.28s", coordsBuf, "LEFT", L.print(rel));
	sprintf(trackBuf, "%s %s: %-25.25s %-7.7s", trackBuf, "LEFT", L.getEnglishTrackingResultForPose(), L.getEnglishPoseValidity());
	sprintf(rotBuf, "%s %s qw:%.2f qx:%.2f qy:%.2f qz:%.2f", rotBuf, "LEFT", L.ROT.w, L.ROT.x, L.ROT.y, L.ROT.z);

	sprintf(bufL, "hand=%s handid=%d trigger=%f padx=%f pady=%f", "LEFT", 1, L.trigger, L.dPadX, L.dPadY);

	sprintf(coordsBuf, "%s %s: %-28.28s", coordsBuf, "RIGHT", R.print(rel));
	sprintf(trackBuf, "%s %s: %-25.25s %-7.7s", trackBuf, "RIGHT", R.getEnglishTrackingResultForPose(), R.getEnglishPoseValidity());
	sprintf(rotBuf, "%s %s qw:%.2f qx:%.2f qy:%.2f qz:%.2f", rotBuf, "RIGHT", R.ROT.w, R.ROT.x, R.ROT.y, R.ROT.z);

	sprintf(bufR, "hand=%s handid=%d trigger=%f padx=%f pady=%f", "RIGHT", 1, R.trigger, R.dPadX, R.dPadY);

	printf("\nCOORDS-- %s", coordsBuf); 
	printf("\nTRACK-- %s", trackBuf);
	printf("\nROT-- %s", rotBuf);

	if (L.valid) printf("\nANALOG-- %s", bufL);
	if (R.valid) printf("  %s", bufR);
}

void Human::calcHeadShoulderOffset(ViveController con, ViveDevice H) {
	shoulderPos.ySet(con.XYZ.y());
	shoulderPos.zSet(con.XYZ.z() - armLength);
	shoulderPos.xSet(H.XYZ.x() - headToArmAcross);

	for (int i = 0; i < 3; i++) {
		headShoulderDistance.v[i] = shoulderPos.v[i] - H.XYZ.v[i];
	}
}

Human::Human(HumanName n) {
	switch (n) {
		case Albert: {
			armLength = .8;
			headToArmAcross = .26;
		}
		case Paulina: {
			armLength = .75;
			headToArmAcross = .18;
		}
		case Sam: {
			armLength = .82;
			headToArmAcross = .25;
		}
		case John: {
			armLength = .8;
			headToArmAcross = .26;
		}
		default : {
			armLength = .78;
			headToArmAcross = .23;
		}
	}
}

RobotArm::RobotArm(float humanArmLength, const char* serial_port) {
	human2ArmConversion = (upperArmLength + foreArmLength + handLength) / humanArmLength;

	base.init(0, InverseK.b2a(0.0), InverseK.b2a(180.0));
	upperarm.init(upperArmLength, InverseK.b2a(0.0), InverseK.b2a(180.0));
	forearm.init(foreArmLength, InverseK.b2a(0.0), InverseK.b2a(180.0));
	hand.init(handLength, InverseK.b2a(0.0), InverseK.b2a(180.0));

	InverseK.attach(base, upperarm, forearm, hand);

	this->SP = new Serial(serial_port);
	if (this->SP->IsConnected())
		printf("We're connected\n");
}

void RobotArm::calcHandPosition(ViveController R) {
	for (int i = 0; i < 3; i++) {
		handPosition.v[i] = R.relativeXYZ.v[i] * (human2ArmConversion /20);
	}
}

void RobotArm::calcAngles(ViveController R) {
	std::cout << InverseK.solve(R.relativeXYZ.y(), R.relativeXYZ.z(), R.relativeXYZ.x(), shoulderAngle, upperArmAngle, foreArmAngle, handAngle) << std::endl;
	gripAngle = 180 * (1-R.trigger);
}

bool RobotArm::send(int base, int shoulder, int elbow, int wrist, int grip) {

	int length;
	char to_send[6];
	to_send[0] = (char)base;
	to_send[1] = (char)shoulder;
	to_send[2] = (char)elbow;
	to_send[3] = (char)wrist;
	to_send[4] = (char)grip;
	to_send[5] = (char)255;
	return (bool)SP->WriteData(to_send, 6);

}

RobotArm::~RobotArm()
{
	this->SP->~Serial();
}