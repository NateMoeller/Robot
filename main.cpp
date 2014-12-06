#include "Aria.h"

#include <iostream>
#include <fstream>
using namespace std;

#define PI 3.14159265

//global laser distances and angles
double laser_dist[900];
double laser_angle[900];

void getLaserReadings(ArSick &sick);



void getLaserReadings(ArSick &sick){
	cout << "calling getLaserDistances..." << endl;
	std::list<ArSensorReading *> *laserReadings;
	std::list<ArSensorReading *>::iterator it;
	int count = 1;
	while (count<10000){
		laserReadings = (list<ArSensorReading *, allocator<ArSensorReading *> > *)sick.getRawReadings();//CurrentBuffer..
		while (laserReadings == NULL){
			laserReadings = (list<ArSensorReading *, allocator<ArSensorReading *> > *)sick.getRawReadings();
		}
		int t = 0;
		for (it = laserReadings->begin(); it != laserReadings->end(); it++){
			//cout << "t: " << t << endl;
			laser_dist[t] = (*it)->getRange();
			laser_angle[t] = -90 + t;
			//cout << "laser angle: " << laser_angle[t] << " laser dist.: " << laser_dist[t] <<" "<<"\n";
			t++;
		}
		cout << "count: " << count << endl; //for some reason this line needs to be here
		count++;
	}
}

int main(int argc, char **argv)
{
	Aria::init();
	ArSimpleConnector connector(&argc, argv);

	//robot and laser scanner variables
	ArRobot robot;
	ArSick sick;
	int t, cnt;
	cnt = 1;
	int config = 0; //start off not knowing where we are

	ofstream laserData;
	laserData.open("laserData.txt");
	
	bool foundP0 = false;
	bool foundP1 = false;
	bool foundP2 = false;
	bool foundP3 = false;
	bool foundP4 = false;

	//key handler variables
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

	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();
	robot.comInt(ArCommands::SOUNDTOG, 0);

	/*
	//start rotating
	robot.lock();
	robot.setVel(0.0);
	robot.setRotVel(1.0);
	robot.move(0.0);
	robot.unlock();
	*/

	
	//move the robot
	ArActionConstantVelocity constantVelocity("Constant Velocity", 70);
	robot.addAction(&constantVelocity, 50);

	int prevP0 = 0;
	int p0 = 0;
	int p1 = 0;

	double p2x = 0;
	double p4x = 0;
	double size = 0;

	int SPOT_SIZE = 1000;

	bool foundSpot = false;
	while (!foundSpot){
		//take a reading
		getLaserReadings(sick);
		for (t = 0; t<181; t++){
			cout << "laser angle: " << laser_angle[t] << " laser dist.: " << laser_dist[t] << " " << "\n";
			laserData << "laser angle: " << laser_angle[t] << " laser dist: " << laser_dist[t] << " " << "\n";
			
			double x = laser_dist[t] * cos(laser_angle[t] * PI / 180);
			double y = laser_dist[t] * sin(laser_angle[t] * PI / 180);
			//laserData << "x: " << x << " y: " << y << endl;
			
			if (config == 1 || config == 0){
				//search for the front of the first box
				//less than 90 because we are going to park on the right side
				if (!foundP0 && t < 90){
					if (laser_dist[t] > laser_dist[t + 1] && (laser_dist[t] - laser_dist[t + 1]) > 5){
						laserData << "p0: " << laser_angle[t] << "\n" << endl;
						p0 = laser_angle[t];
						laserData << "x: " << x << " y: " << y << endl;
						foundP0 = true;
					}
				}
				//find p1
				if (foundP0 && !foundP1 && t < 90){
					if (laser_dist[t] < laser_dist[t + 1]){
						laserData << "p1: " << laser_angle[t] << "\n" << endl;
						p1 = laser_angle[t];
						laserData << "x: " << x << " y: " << y << endl;
						foundP1 = true;
					}
				}

			}
			else if (config == 2){
				//determine the end of the first box, and the beginning of the 2nd box
				//find p2
				if (!foundP2 && (laser_dist[t + 1] - laser_dist[t]) > 50){
					laserData << "p2: " << laser_angle[t] << " x: " << x << " y: " << y << endl;
					p2x = x;
					foundP2 = true;
				}

				//find P3
				if (foundP2 && !foundP3 && t < 90){
					if (laser_dist[t] > laser_dist[t + 1]){
						laserData << "p3: " << laser_angle[t] << " x: " << x << " y: " << y << endl;
						foundP3 = true;
					}
				}

				//find P4
				if (foundP2 && foundP3 && !foundP4 && t < 90){
					if (laser_dist[t] < laser_dist[t + 1]){
						laserData << "p4: " << laser_angle[t] << " x: " << x << " y: " << y << endl;
						p4x = x;
						foundP4 = true;
					}
				}

				if (foundP2 && foundP3 && foundP4){
					//calculate the size of the spot (x axis)
					size = (p4x - p2x);
					if (size > SPOT_SIZE){
						foundSpot = true;
					}
					else{
						laserData << "spot found. too small. keep going" << endl;
						config = 1;
						foundP0 = false;
						foundP1 = false;
						foundP2 = false;
						foundP3 = false;
						foundP4 = false;
						prevP0 = 0;
					}
					
				}
			}
		}

		//determine if we are in front of the first box, next to the first box, or next to the 3rd box
		if (foundP0 && foundP1 && p0 < prevP0 && config != 2){
			laserData << "config1" << endl;
			config = 1;
		}
		else if (foundP0 && foundP1 && p0 > prevP0){
			//the robot is in front of the box.
			laserData << "config2" << endl;
			config = 2;
		}

		prevP0 = p0;
		foundP0 = false;
		foundP1 = false;
		laserData << "\n\n" << endl;
	}
	
	/*
	laserData << "start parking..." << endl;
	robot.lock();
	robot.stop();
	robot.move(p2x);
	robot.stop();
	robot.move(p4x);
	robot.unlock();
	ArUtil::sleep(5000);

	
	//turning code
	laserData << "first turn" << endl;
	robot.lock();
	robot.setVel2(-200, -80);
	robot.unlock();
	ArTime start;
	start.setToNow();
	while (1){
		robot.lock();
		if (start.mSecSince() > 4000)
		{
			robot.unlock();
			break;
		}
		robot.unlock();
		ArUtil::sleep(50);
	}
	robot.lock();
	robot.stop();
	robot.unlock();

	ArUtil::sleep(5000);
	//printf("backward\n");  
	//robot.lock();
	//robot.setVel2(-50, -50);
	//robot.unlock();
	//ArUtil::sleep(5000);
	
	
	
	laserData << "second turn" << endl;
	robot.lock();
	robot.setVel2(-80, -200);
	robot.unlock();
	start.setToNow();
	while (1){
		robot.lock();
		if (start.mSecSince() > 4000)
		{
			robot.unlock();
			break;
		}
		robot.unlock();
		ArUtil::sleep(50);
	}
	robot.lock();
	robot.stop();
	robot.unlock();
	*/

	//robot.runAsync(false);
	/*
	//move the robot
	robot.lock();
	robot.setVel(10.0);
	robot.move(2000.0);
	robot.unlock();
	*/

	/*
	//this will set the robot to go in a straight line at a constant velocity (must set runAsync to true)
	ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
	robot.addAction(&constantVelocity, 50);
	*/
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();

	laserData.close();
	Aria::exit(0);
	return 0;
}
