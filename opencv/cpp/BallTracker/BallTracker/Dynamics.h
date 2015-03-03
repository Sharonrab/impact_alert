
#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <fstream>
// Include OpenCV libraries
// TODO
#include <opencv2/core.hpp>

#pragma once
using namespace std;
using namespace cv;

class Dynamics
{
public:
	Dynamics(double initialSpeed, double initialHeight, double angle);
	void runSimulation();
	void calculateCurrentPositions();
	void calculateCurrentSpeeds();
	void calculateAirResistance();
	bool onlyCalculateIdealPath();
	void Init(double initialSpeed, double initialHeight, double angle);
	void EstimateVelocityComponents(cv::Point2i currentPnt, cv::Point2i prevPnt,int currentBallRadius, int prevBallRadius, double &V, double &verticalAngle ,double &horizontalAngle);

	//------------------------VARIABLES-------------------------------
private:
	//GRAVITY
	double gravity; // m/s^2 (down is defined to be positive)
	//AIR
	double airDensity; // Air density in kg/m^3
	double airResistance; // Air resistance at current moment in N
	double dragCoefficient;
	//ANGLE
	double angle; // Angle throw in degrees
	//AREA
	double area; // Cross sectional area of object
	//SPEED
	double initialSpeed;
	double currentHorizontalSpeed; // x component of object
	double currentVerticalSpeed; // y component of object
	double currentAirSpeed; // actual speed of object
	//POSITION
	double initialHeight;
public:
	std::vector<double> height;
	std::vector<double> distance;
	std::vector<double> idealHeight;
	std::vector<double> idealDistance;

	//MASS
	double mass;
	//TIME
	double time ; // Time that has passed in simulation
	double idealTime ; //Time passed in ideal simulation. 
	int timeInterval; //Used to access height and distance array
	double PHYSICS_REFRESH_RATE ;  //Time interval calculated
	ofstream logfile;
  

	int iteration;

	//--------------------------GETTERS AND SETTERS--------------------------
public:
	void setGravity(double gravity)
	{
		this->gravity = gravity;
	}

	void setAirDensity(double airDensity)
	{
		this->airDensity = airDensity;
	}

	void setDragCoefficient(double dragCoefficient)
	{
		this->dragCoefficient = dragCoefficient;
	}

	void setArea(double area)
	{
		this->area = area;
	}

	void setMass(double mass)
	{
		this->mass = mass;
	}

	int getArraySize()
	{
		return min(height.size(), distance.size());
	}

	int getIdealArraySize()
	{
		return min(idealHeight.size(), idealDistance.size());
	}

	double getHeight(int i)
	{
		return height[i];
	}

	double getMaxHeight()
	{
		return 0;// max(height);
	}

	double getMaxDistance()
	{
		return distance[distance.size()];
	}

	double getMaxIdealHeight()
	{
		return 0;// Collections.max(idealHeight);
	}

	double getMaxIdealDistance()
	{
		return idealDistance[idealDistance.size()];
	}

	double getLargestArrayValue(bool isIdealPath)
	{
		if (isIdealPath)
		{
			std::vector<double> myVector(std::max_element(std::max_element(idealHeight.begin(), idealHeight.end()),
				std::max_element(idealDistance.begin(), idealDistance.end())),
				std::max_element(std::max_element(height.begin(), height.end()),
				std::max_element(distance.begin(), distance.end())));
//			double temp =  std::max_element(myVector.begin(), myVector.end());
			return 1;
		}
		else
		{
			//std::vector<double> myVector = std::max_element(height.begin(), height.end()), std::max_element(distance.begin(), distance.end());
			return 0;//std::max_element(myVector);
		}
	}

	double getDistance(int i)
	{
		return distance[i];
	}

	double getIdealHeight(int i)
	{
		return idealHeight[i];
	}

	double getIdealDistance(int i)
	{
		return idealDistance[i];
	}

	int getTimeInterval()
	{
		return timeInterval;
	}
	double getTime()
	{
		return time;
	}

	double getIdealTime()
	{
		return idealTime;
	}

	double getAirSpeed()
	{
		return currentHorizontalSpeed;
	}


};