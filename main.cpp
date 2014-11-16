#include "Aria.h"

#include <iostream>
using namespace std;



int main(int argc, char **argv)
{
	Aria::init();
	ArSimpleConnector connector(&argc, argv);
	ArRobot robot;
	ArSick sick;
	int t, cnt;
	cnt = 1;
	double laser_dist[900];
	double laser_angle[900];
	std::list<ArSensorReading *> *readings;
	std::list<ArSensorReading *>::iterator it;

	ArKeyHandler keyHandler; 
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);

	if (!Aria::parseArgs() || argc > 1){
		Aria::logOptions();
		Aria::shutdown();
		exit(1);
	}
	// add the laser to the robot
	robot.addRangeDevice(&sick);

	//connect to robot
	if (!connector.connectRobot(&robot)){
		printf("Could not connect to robot... exiting\n");
		Aria::shutdown();
		return 1;
	}

	// start the robot running, true so that if we lose connection the run stops
	robot.runAsync(true);
	puts("Robot Running");

	//toggle true to false when actually connecting to the robot
	sick.configureShort(true, ArSick::BAUD38400, ArSick::DEGREES180, ArSick::INCREMENT_ONE);
	// set up the laser before handing it to the laser mode
	connector.setupLaser(&sick); //this line throws an exception in debugging mode

	sick.runAsync();

	if (!sick.blockingConnect()){
		printf("Could not connect to SICK laser... exiting\n");
		Aria::shutdown();
		return 1;
	}

	
	cnt = 1;
	while (cnt<10000){
		readings = (list<ArSensorReading *, allocator<ArSensorReading *> > *)sick.getRawReadings();//CurrentBuffer..
		while (readings == NULL){
			readings = (list<ArSensorReading *, allocator<ArSensorReading *> > *)sick.getRawReadings();
		}
		t = 0;
		for (it = readings->begin(); it != readings->end(); it++){
			//cout << "t: " << t << endl;
			laser_dist[t] = (*it)->getRange();
			laser_angle[t] = -90 + t;
			//cout << "laser angle: " << laser_angle[t] << " laser dist.: " << laser_dist[t] <<" "<<"\n";
			t++;
		}
		cout << "count: " << cnt << endl; //for some reason this line needs to be here
		cnt++;

	}
	
	for (t = 0; t<181; t++){
		cout << "laser angle: " << laser_angle[t] << " laser dist.: " << laser_dist[t] << " " << "\n";
	}


	/*
	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();
	robot.comInt(ArCommands::SOUNDTOG, 0);

	//this will set the robot to go in a straight line at a constant velocity (must set runAsync to true)
	ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
	robot.addAction(&constantVelocity, 50);
	*/
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();

	/*
	//run the robot
	robot.runAsync(false);
	//move the robot
	printf("Telling the robot to move forwards five meters\n");
	robot.lock();
	robot.move(5000);
	robot.unlock();
	*/


	Aria::exit(0);
	return 0;
}
