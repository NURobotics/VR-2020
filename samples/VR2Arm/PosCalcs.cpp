#include "PosCalcs.h"

//using namespace std;

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

void Human::calcHeadShoulderOffset(ViveController con, ViveDevice H) {
	shoulderPos.ySet(con.XYZ.y());
	shoulderPos.zSet(con.XYZ.z() + armLength);
	shoulderPos.xSet(std::abs(H.XYZ.x() - headToArmAcross));

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

void AllViveDevices::calcRelHandPos(Human Hu) {
	R.relativeXYZ.xSet(R.XYZ.x() - H.XYZ.x() + Hu.headShoulderDistance.x());
	R.relativeXYZ.ySet(R.XYZ.y() - H.XYZ.y() - Hu.headShoulderDistance.y());
	R.relativeXYZ.zSet(-R.XYZ.z() + H.XYZ.z() + Hu.headShoulderDistance.z());

	L.relativeXYZ.xSet(R.XYZ.x() - H.XYZ.x() - Hu.headShoulderDistance.x());
	L.relativeXYZ.ySet(R.XYZ.y() - H.XYZ.y() - Hu.headShoulderDistance.y());
	L.relativeXYZ.zSet(-R.XYZ.z() + H.XYZ.z() + Hu.headShoulderDistance.z());
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

void RobotArm::calcHandPosition(ViveController C) {
	for (int i = 0; i < 3; i++) {
		handPosition.v[i] = C.relativeXYZ.v[i] * human2ArmConversion;
	}
}

void RobotArm::calcAngles(ViveController C) {
	float positionLength = sqrt(handPosition.v[0] * handPosition.v[0] + handPosition.v[1] * handPosition.v[1] + handPosition.v[2] * handPosition.v[2]);
	if (positionLength > armMaxLength) {
		float scaleFactor = armMaxLength / positionLength;
		for (int i = 0; i < 3; i++) {
			handPosition.v[i] *= scaleFactor;
		}
	}

	inverseKin();
	gripAngle = 180 * (1-C.trigger);
}

float jointRads2Deg(float radAngle) {
	return std::min(std::max(degrees(radAngle), 0.0), 180.0);
}

void RobotArm::inverseKin() {
    float x = handPosition.z();
    float y = -handPosition.x();
    float z = handPosition.y();

	float r = std::sqrt(x * x + y * y);

    z = std::max(z, (float) 0.020);

    const float baseRad = atan2(x, y);

    // i = global wrist angle
    for (float i = 0; i <= APPLE_PI; i+=DEG_TO_RAD) {
        const float wrist_y = r - cos(i) * handLength;
        const float wrist_z = z - sin(i) * handLength;

        /* Shoulder to wrist distance ( AKA sw ) */
        const float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
        const float s_w_sqrt = sqrt(s_w);

        /* s_w angle to ground */
        const float a1 = atan2(wrist_y, wrist_z);

        /* s_w angle to humerus */
        const float a2 = acos((foreArmLenSq - upperArmLenSq + s_w) / (2.0 * s_w_sqrt * foreArmLength));

        const float elbowRad = acos((upperArmLenSq + foreArmLenSq - s_w) / (2.0 * upperArmLength * foreArmLength));
        const float shoulderRad = HALF_PI_3 - (a1 + a2) - elbowRad;
        const float wristRad = i - (elbowRad + shoulderRad) + HALF_PI_3;

        if (!isnan(shoulderRad) && !isnan(elbowRad) && !isnan(wristRad)) {
            shoulderAngle = 180 - jointRads2Deg(shoulderRad);
            elbowAngle = jointRads2Deg(elbowRad);
            wristAngle = 180 - jointRads2Deg(wristRad);
			baseAngle = 180 - jointRads2Deg(baseRad);
            break;
        }
    }
}

bool RobotArm::sendAngles(int base, int shoulder, int elbow, int wrist, int grip) {
	char to_send[6];
	to_send[0] = (char)base;
	to_send[1] = (char)shoulder;
	to_send[2] = (char)elbow;
	to_send[3] = (char)wrist;
	to_send[4] = (char)grip;
	to_send[5] = (char)255;

	if (ONLINE) {
		send(server, to_send, 6, 0);
	}
	else {
		return (bool)SP->WriteData(to_send, 6);
	}
}

RobotArm::RobotArm(float humanArmLength, float ul, float fl, float hl, const char* serial_port, bool online) :
	upperArmLength(ul), foreArmLength(fl), handLength(hl) {
	armMaxLength = (upperArmLength + foreArmLength + handLength) * .99;
	human2ArmConversion = armMaxLength / humanArmLength;
	ONLINE = online;

	if (ONLINE) {
		WSAStartup(MAKEWORD(2, 0), &wsaData);
		server = socket(AF_INET, SOCK_STREAM, 0);
		addr.sin_addr.s_addr = inet_addr("34.224.214.122");
		addr.sin_family = AF_INET;
		addr.sin_port = htons(4444);

		connect(server, (SOCKADDR*)&addr, sizeof(addr));
	}
	else {
		this->SP = new Serial(serial_port);
		if (this->SP->IsConnected())
			printf("We're connected\n");
	}

	upperArmLenSq = upperArmLength * upperArmLength;
	foreArmLenSq = foreArmLength * foreArmLength;
	handLenSq = handLength * handLength;

	handPosition.xSet(0.0); handPosition.ySet(0.0); handPosition.zSet(0.0);
}

RobotArm::~RobotArm() {
	if (ONLINE) {
		closesocket(server);
		WSACleanup();
	}
	else {
		this->SP->~Serial();
	}
}
