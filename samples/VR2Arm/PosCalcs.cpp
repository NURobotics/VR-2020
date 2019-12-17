#include "PosCalcs.h"

#define APPLE_PI 3.14159265359

using namespace std;

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
	armMaxLength = upperArmLength + foreArmLength + handLength;
	human2ArmConversion = armMaxLength / humanArmLength;

	this->SP = new Serial(serial_port);
	if (this->SP->IsConnected())
		printf("We're connected\n");

	upperArmLenSq = upperArmLength * upperArmLength;
	foreArmLenSq = foreArmLength * foreArmLength;
	handLenSq = handLength * handLength;

	handPosition.xSet(0.0); handPosition.ySet(0.0); handPosition.zSet(0.0);
}

void RobotArm::calcHandPosition(ViveController R) {
	for (int i = 0; i < 3; i++) {
		handPosition.v[i] = R.relativeXYZ.v[i] * human2ArmConversion;
	}
}

void RobotArm::calcAngles(ViveController R) {
	float positionLength = sqrt(handPosition.v[0] * handPosition.v[0] + handPosition.v[1] * handPosition.v[1] + handPosition.v[2] * handPosition.v[2]);
	if (positionLength > armMaxLength) {
		float scaleFactor = armMaxLength / positionLength;
		for (int i = 0; i < 3; i++) {
			//handPosition.v[i] *= scaleFactor;
		}
	}

	//inverseKin();
	IK2();
	gripAngle = 180 * (1-R.trigger);
}

void RobotArm::inverseKin() {
    float x = handPosition.x();
    float y = handPosition.z();
    float z = handPosition.y();

    x = min(max(x, (float) -0.150), (float) 0.150);
    y = min(max(y, (float) 0.050), (float) 0.200);
    z = min(max(z, (float) 0.020), (float) 0.225);
	cout << x << " " << y << " " << z << endl;

	/* Base angle and radial distance from x,y coordinates */
	baseAngle = atan2(x, y);
	y = sqrt((x * x) + (y * y));

	/* Wrist position */
	float wrist_z = z - baseHeight;
	float wrist_y = y - handLength;

	/* Shoulder to wrist distance ( AKA sw ) */
	float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
	float s_w_sqrt = sqrt(s_w);

	/* s_w angle to ground */
	float a1 = atan2(wrist_z, wrist_y);

	/* s_w angle to humerus */
	float a2 = acos(((upperArmLenSq - foreArmLenSq) + s_w) / (2.0 * upperArmLength * s_w_sqrt));

	/* shoulder angle */
	float shl_angle_r = a1 + a2;
	shoulderAngle = 180.0 * shl_angle_r / APPLE_PI;

	/* elbow angle */
	float elb_angle_r = acos((upperArmLenSq + foreArmLenSq - s_w) / (2.0 * upperArmLength * foreArmLength));
	float elb_angle_d = 180.0 * elb_angle_r / APPLE_PI;
	elbowAngle = -(180.0 - elb_angle_d);

	/* wrist angle */
	wristAngle = - elbowAngle - shoulderAngle;

	shoulderAngle = 90.0 -((shoulderAngle - 90.0)*.633);
	elbowAngle = 90.0 + ((elbowAngle - 90.0) * .633);
	wristAngle = 90.0 + (wristAngle * .633);
}

bool inRange(float num) {
    return (num >= 0) && (num <= APPLE_PI);
}

bool _cosrule(float opposite, float adjacent1, float adjacent2, float& angle) {
	float delta = 2 * adjacent1 * adjacent2;

	if (delta == 0) return false;

	float cos = (adjacent1*adjacent1 + adjacent2*adjacent2 - opposite*opposite) / delta;

	if ((cos > 1) || (cos < -1)) return false;

	angle = acos(cos);

	return true;
}

bool RobotArm::_solve(float x, float y, float phi, float& shoulder, float& elbow, float& wrist) {
	// Adjust coordinate system for base as ground plane
	float _r = sqrt(x*x + y*y);
	float _theta = atan2(y, x);
	float _x = _r * cos(_theta - APPLE_PI / 2);
	float _y = _r * sin(_theta - APPLE_PI / 2);
	float _phi = phi - APPLE_PI / 2;

	// Find the coordinate for the wrist
	float xw = _x - handLength * cos(_phi);
	float yw = _y - handLength * sin(_phi);

	// Get polar system
	float alpha = atan2(yw, xw);
	float R = sqrt(xw*xw + yw*yw);

	// Calculate the inner angle of the shoulder
	float beta;
	if(!_cosrule(foreArmLength, R, upperArmLength, beta)) return false;

	// Calcula the inner angle of the elbow
	float gamma;
	if(!_cosrule(R, upperArmLength, foreArmLength, gamma)) return false;

	// Solve the angles of the arm
	float _shoulder, _elbow, _wrist;
	_shoulder = alpha - beta;
	_elbow = 3*APPLE_PI/2 - gamma;
	_wrist = _phi - _shoulder - _elbow;

	// Check the range of each hinge
	if (!inRange(_shoulder) || !inRange(_elbow) || !inRange(_wrist)) {
		// If not in range, solve for the second solution
		_shoulder += 2 * beta;
		_elbow *= -1;
		_wrist = _phi - _shoulder - _elbow;

		// Check the range for the second solution
		if (!inRange(_shoulder) || !inRange(_elbow) || !inRange(_wrist)) return false;
	}

	// Return the solution
	shoulder = 180 * _shoulder / APPLE_PI;
	elbow = 180 * _elbow / APPLE_PI;
	wrist = 180 * _wrist / APPLE_PI;

	return true;
}

bool RobotArm::_solve(float x, float y, float& shoulder, float& elbow, float& wrist) {
	if (_solve(x, y, -2.0*APPLE_PI, shoulder, elbow, wrist)) return true;
	for (float phi = -2.0*APPLE_PI; phi < 2.0*APPLE_PI; phi += 0.01745329251) {
		if (_solve(x, y, phi, shoulder, elbow, wrist)) {
			return true;
		}
	}

	return false;
}

void RobotArm::IK2() {
    float x = handPosition.x();
    float y = handPosition.z();
    float z = handPosition.y();

    float _r = sqrt(x*x + y*y);
	float _base = atan2(y, x);

	// Check the range of the base
	if (!inRange(_base)) {
		// If not in range, flip the angle
		_base += (_base < 0) ? APPLE_PI : -APPLE_PI;
		_r *= -1;
	}

	// Solve XY (RZ) for the arm plane
	if (!_solve(_r, z - upperArmLength, shoulderAngle, elbowAngle, wristAngle)) return;

	// If there is a solution, return the angles
	baseAngle = _base;
}

bool RobotArm::send(int base, int shoulder, int elbow, int wrist, int grip) {
	char to_send[6];
	to_send[0] = (char)base;
	to_send[1] = (char)shoulder;
	to_send[2] = (char)elbow;
	to_send[3] = (char)wrist;
	to_send[4] = (char)grip;
	to_send[5] = (char)255;

	return (bool)SP->WriteData(to_send, 6);
}

RobotArm::~RobotArm() {
	this->SP->~Serial();
}
