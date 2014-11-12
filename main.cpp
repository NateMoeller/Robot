#include "Aria.h"

ArRobot robot;
ArSick sick;

using namespace std;
int main(int argc, char **argv)
{
	Aria::init();
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobotConnector robotConnector(&argParser, &robot);

	// Always try to connect to the first laser:
	argParser.addDefaultArgument("-connectLaser");

	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (argParser.checkHelpAndWarnUnparsed())
		{
			// -help not given, just exit.
			Aria::logOptions();
			Aria::exit(1);
			return 1;
		}
	}


	// Trigger argument parsing
	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}

	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);

	puts("Our initial program to control the robot");

	//ArSonarDevice sonar;
	//robot.addRangeDevice(&sonar);


	//run the robot
	robot.runAsync(false);

	// try to connect to laser. if fail, warn but continue, using sonar only
	if (!laserConnector.connectLasers()){
		ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only.");
	}


	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();
	robot.comInt(ArCommands::SOUNDTOG, 0);

	//move the robot
	printf("Telling the robot to move forwards five meters\n");
	robot.lock();
	robot.move(5000);
	robot.unlock();

	//robot.clearDirectMotion();
	//printf("Now using actions\n");
	//this will set the robot to go in a straight line at a constant velocity (must set runAsync to true)
	//ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
	//robot.addAction(&constantVelocity, 50);

	
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();

	Aria::exit(0);
	return 0;
}