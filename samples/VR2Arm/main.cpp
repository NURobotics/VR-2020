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

#include "shared/lodepng.h"
#include "shared/Matrices.h"
#include "shared/pathtools.h"

#if defined(POSIX)
#include "unistd.h"
#endif

#include <VR2Arm\LighthouseTracking.h>


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	InitFlags flags;
	AllViveDevices ViveSystem;
	Human Operator(Paulina);
	RobotArm claw(Operator.armLength, "\\\\.\\COM4");

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
				claw.send((int)claw.shoulderAngle, (int)claw.upperArmAngle, (int)claw.foreArmAngle, (int)claw.handAngle, claw.gripAngle);
				//claw.send((int)0, (int)45, (int)180, (int)45, claw.gripAngle);
				std::cout << std::endl << claw.handPosition.x() << std::endl << claw.handPosition.y() << std::endl << claw.handPosition.z() << std::endl << std::endl;
			}
			
			Sleep(1);
		}
		delete lighthouseTracking;
	}
	return EXIT_SUCCESS;
}
