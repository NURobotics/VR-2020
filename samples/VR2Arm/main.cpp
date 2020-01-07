//========= Copyright Valve Corporation ============//

#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#if defined( OSX )
#include <Foundation/Foundation.h>
#include <AppKit/AppKit.h>
#include <OpenGL/glu.h>
// Apple's version of glut.h #undef's APIENTRY, redefine it
#define APIENTRY
#else
#include <GL/glu.h>
#endif
#include <stdio.h>
#include <string>
#include <cstdlib>

#include <openvr.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "shared/lodepng.h"
#include "shared/Matrices.h"
#include "shared/pathtools.h"

#if defined(POSIX)
#include "unistd.h"
#endif

#include <VR2Arm\LighthouseTracking.h>

using namespace cv;

const double pi = 3.1415926;
int mouseX;
int mouseY;
float mouseXCon, mouseYCon;

const int frameHeight = 720;
const int frameWidth = 1280;
const int halfHeight = frameHeight / 2;
const int halfWidth = frameWidth / 2;
const float displaySF = 1000;

const Scalar red = Scalar(0, 0, 255);
const Scalar blue = Scalar(255, 0, 0);
const Scalar green = Scalar(0, 255, 0);

void calcMCon() {
	mouseXCon = (mouseX - halfWidth) / displaySF;
	mouseYCon = (halfHeight - mouseY) / displaySF;
}

static void onMouse(int event, int x, int y, int, void*)
{
	mouseX = x;
	mouseY = y;
	calcMCon();
}

template <typename T >
T cosD(T angle) {
	return cos(angle * pi / 180.0);
}
template <typename T >
T sinD(T angle) {
	return sin(angle * pi / 180.0);
}

void drawArmSym(RobotArm& r) {

	Mat image = Mat::zeros(frameHeight, frameWidth, CV_8UC3);

	const float dispShouldAngle = r.shoulderAngle + 180;
	const float dispElbowAngle = dispShouldAngle - r.elbowAngle + 180;
	const float dispWristAngle = dispElbowAngle + r.wristAngle - 90;

	int x2 = r.upperArmLength * displaySF * cosD(dispShouldAngle) + halfWidth;
	int y2 = r.upperArmLength * displaySF * sinD(dispShouldAngle) + halfHeight;
	int x3 = r.foreArmLength * displaySF * cosD(dispElbowAngle) + x2;
	int y3 = r.foreArmLength * displaySF * sinD(dispElbowAngle) + y2;
	int x4 = r.handLength * displaySF * cosD(dispWristAngle) + x3;
	int y4 = r.handLength * displaySF * sinD(dispWristAngle) + y3;

	circle(image, Point(halfWidth, halfHeight), r.upperArmLength * displaySF, blue, 1, 8);
	line(image, Point(halfWidth, halfHeight), Point(x2, y2), blue, 6, 8);
	circle(image, Point(x2, y2), r.foreArmLength * displaySF, green, 1, 8);
	line(image, Point(x2, y2), Point(x3, y3), green, 4, 8);
	circle(image, Point(x3, y3), r.handLength * displaySF, red, 1, 8);
	line(image, Point(x3, y3), Point(x4, y4), red, 2, 8);

	putText(image, "X: " + std::to_string(mouseXCon * displaySF) + "mm Y: " + std::to_string(mouseYCon * displaySF) + "mm",
		Point(halfWidth - 240, 640), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
	putText(image, "Shoulder X: " + std::to_string(x2 - halfWidth) + "mm    Elbow X:    " + std::to_string(x3 - halfWidth) + "mm    Wrist X:    " + std::to_string(x4 - halfWidth) + "mm",
		Point(halfWidth - 240, 660), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
	putText(image, "Shoulder Y: " + std::to_string(halfHeight - y2) + "mm    Elbow Y:    " + std::to_string(halfHeight - y3) + "mm    Wrist Y:    " + std::to_string(halfHeight - y4) + "mm",
		Point(halfWidth - 240, 680), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
	putText(image, "Shoulder Angle: " + std::to_string(r.shoulderAngle) + "    Elbow Angle:    " + std::to_string(r.elbowAngle) + "    Wrist Angle:    " + std::to_string(r.wristAngle),
		Point(halfWidth - 240, 700), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);

	drawMarker(image, Point(mouseX, mouseY), Scalar(255, 255, 255), MARKER_DIAMOND, 25, 2);

	imshow("Arm", image);
	char key = waitKey(1);
	if (key == '6') { mouseX++; calcMCon(); }
	if (key == '4') { mouseX--; calcMCon(); }
	if (key == '2') { mouseY++; calcMCon(); }
	if (key == '8') { mouseY--; calcMCon(); }
}

void ArmTest(AllViveDevices ViveSystem, RobotArm claw) {
	claw.send(0, 90, 180, 90, 0);
	namedWindow("Arm");
	setMouseCallback("Arm", onMouse);
	float x = 0;

	Sleep(2000);
	while (1) {
		claw.handPosition.xSet(.25);
		claw.handPosition.ySet(mouseXCon);
		claw.handPosition.zSet(mouseYCon);

		claw.calcAngles(ViveSystem.R);
		//claw.send((int)0, (int)90, (int)180, (int)0, claw.gripAngle);
		claw.send((int)claw.baseAngle, (int)claw.shoulderAngle, (int)claw.elbowAngle, (int)claw.wristAngle, claw.gripAngle);
		drawArmSym(claw);

		std::cout << std::endl << claw.handPosition.x() << std::endl << claw.handPosition.y() << std::endl << claw.handPosition.z() << std::endl << std::endl;
	}
}

void printCoords(ViveController R, Human O) {
	Mat image = Mat::zeros(frameHeight, frameWidth, CV_8UC3);

	putText(image, "XRel: " + std::to_string(R.relativeXYZ.x()) + "m XHSD: " + std::to_string(O.headShoulderDistance.v[0]) + "m",
		Point(halfWidth - 240, 640), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
	putText(image, "YRel: " + std::to_string(R.relativeXYZ.y()) + "m YHSD: " + std::to_string(O.headShoulderDistance.v[1]) + "m",
		Point(halfWidth - 240, 660), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
	putText(image, "ZRel: " + std::to_string(R.relativeXYZ.z()) + "m ZHSD: " + std::to_string(O.headShoulderDistance.v[2]) + "m",
		Point(halfWidth - 240, 680), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);

	imshow("Arm", image);
	char key = waitKey(1);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	InitFlags flags;
	AllViveDevices ViveSystem;
	Human Operator(Paulina);
	RobotArm claw(Operator.armLength, "\\\\.\\COM4", 0.096, 0.0906, 0.096);

	bool isHelp = false;
	bool invert = false;
	bool hasHadFlag = false;
	//for (int x = 0; x < argc; x++)
	//{
	//	char* currString = argv[x];
	//	int len = strlen(currString);
	//	bool isFlag = len > 1 && currString[0] == '-' && '-' != currString[1];

	//	if (!hasHadFlag && isFlag)
	//	{
	//		hasHadFlag = true;
	//		invert = !invert;
	//	}

	//	if (isFlag)
	//		for (int y = 1; y < len; y++)
	//		{
	//			if (currString[y] == 'h')
	//				isHelp = true;
	//			if (currString[y] == 'c')
	//				flags.printCoords = false;
	//			if (currString[y] == 'a')
	//				flags.printAnalog = false;
	//			if (currString[y] == 'e')
	//				flags.printEvents = false;
	//			if (currString[y] == 'i')
	//				flags.printSetIds = false;
	//			if (currString[y] == 'b')
	//				flags.printBEvents = false;
	//			if (currString[y] == 't')
	//				flags.printTrack = false;
	//			if (currString[y] == 'r')
	//				flags.printRotation = false;
	//			if (currString[y] == 'V')
	//				flags.pipeCoords = true;
	//			if (currString[y] == 'O')
	//				invert = !invert;
	//		}

	//	if (!isFlag)
	//	{
	//		if (strcmp("--help", currString) == 0)
	//			isHelp = true;
	//		if (strcmp("--coords", currString) == 0)
	//			flags.printCoords = false;
	//		if (strcmp("--analog", currString) == 0)
	//			flags.printAnalog = false;
	//		if (strcmp("--events", currString) == 0)
	//			flags.printEvents = false;
	//		if (strcmp("--ids", currString) == 0)
	//			flags.printSetIds = false;
	//		if (strcmp("--bevents", currString) == 0)
	//			flags.printBEvents = false;
	//		if (strcmp("--track", currString) == 0)
	//			flags.printTrack = false;
	//		if (strcmp("--rot", currString) == 0)
	//			flags.printRotation = false;
	//		if (strcmp("--visual", currString) == 0)
	//			flags.pipeCoords = true;
	//		if (strcmp("--omit", currString) == 0)
	//			invert = !invert;
	//	}
	//}

	//if (invert)
	//{
	//	flags.printCoords = !flags.printCoords;
	//	flags.printAnalog = !flags.printAnalog;
	//	flags.printEvents = !flags.printEvents;
	//	flags.printSetIds = !flags.printSetIds;
	//	flags.printBEvents = !flags.printBEvents;
	//	flags.printTrack = !flags.printTrack;
	//	flags.printRotation = !flags.printRotation;
	//}

	//if (isHelp)
	//{
	//	printf("\nVive LighthouseTracking Example by Kevin Kellar.\n");
	//	printf("Command line flags:\n");
	//	printf("  -h --help    -> Prints this help text. The \"Only Print\" flags can be combined for multiple types to both print.\n");
	//	printf("  -a --analog  -> Only print analog button data from the controllers. \n");
	//	printf("  -b --bEvents -> Only print button event data. \n");
	//	printf("  -c --coords  -> Only print HMD/Controller coordinates. \n");
	//	printf("  -e --events  -> Only print VR events. \n");
	//	printf("  -i --ids     -> Only print the output from initAssignIds() as the devices are given ids. \n");
	//	printf("  -r --rot     -> Only print the rotation of devices. \n");
	//	printf("  -t --track   -> Only print the tracking state of devices. \n");
	//	printf("  -O --omit    -> Omits only the specified output types (a,b,c,e,i,r,t) rather than including only the specified types.  Useful for hiding only a few types of output. \n");
	//	printf("  -V --visual  -> Streamlines output (coordinates) to be more easily parsed by a visual program. \n");
	//	return EXIT_SUCCESS;
	//}

	//if (flags.pipeCoords)
	//{
	//	flags.printCoords = false;
	//	flags.printAnalog = false;
	//	flags.printEvents = false;
	//	flags.printSetIds = false;
	//	flags.printBEvents = false;
	//	flags.printTrack = false;
	//	flags.printRotation = false;
	//}

	//ArmTest(ViveSystem, claw);

	// Create a new LighthouseTracking instance and parse as needed
	LighthouseTracking* lighthouseTracking = new LighthouseTracking(flags, Operator, ViveSystem);
	if (lighthouseTracking) //null check
	{
		claw.send(0, 90, 180, 90, 0);
		Sleep(2000);
		while (1)
		{
			lighthouseTracking->RunProcedure();
			ViveSystem.calcRelHandPos(Operator);

			//std::cout << ViveSystem.L.calibarated << std::endl << ViveSystem.R.calibarated << std::endl << std::endl;
			if (ViveSystem.R.valid && ViveSystem.R.calibarated) {
				claw.calcHandPosition(ViveSystem.R);
				claw.calcAngles(ViveSystem.R);
				claw.send((int)claw.baseAngle, (int)claw.shoulderAngle, (int)claw.elbowAngle, (int)claw.wristAngle, claw.gripAngle);
				//claw.send((int)0, (int)45, (int)180, (int)45, claw.gripAngle);
				printCoords(ViveSystem.R, Operator);
			}

			Sleep(1);
		}
		delete lighthouseTracking;
	}
	return EXIT_SUCCESS;
}
