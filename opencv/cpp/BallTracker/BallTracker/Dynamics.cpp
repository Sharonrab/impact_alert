#include "Dynamics.h"

#define M_PI       3.14159265358979323846

#define RAD2DEG 180.0/M_PI
#define DEG2RAD M_PI/180.0
//----------------------------CONSTRUCTOR-----------------------------------

Dynamics::Dynamics(double initialSpeed, double initialHeight, double angle)
	
{
	this->angle = (angle)* DEG2RAD;
	this->initialSpeed = initialSpeed;
	this->initialHeight = initialHeight;
	this->time=0;
	this->idealTime=0;
	this->timeInterval=1;//large then 1
	this->PHYSICS_REFRESH_RATE= .01;
	this->gravity = 9.81;
	this->airDensity = 1.225;//for temprature of 15 celcius
	// A used ball experiences a slightly lower drag force since it has a smoother surface. A standard new ball
	//of radius 3.3 cm therefore experiences a backwards force 
	this->dragCoefficient = 0.55;// Cd is about 0.55
	this->mass = 0.057;//  m = 0.057 kg for a standard tennis ball
	this->area =  0.00342;//A = pi R^2 is the cross-sectional area of the ball, R is the radius of the ball

	currentHorizontalSpeed = cos(this->angle) * initialSpeed;
	currentVerticalSpeed = sin(this->angle) * initialSpeed;
	currentAirSpeed = initialSpeed;


	this->height.insert(height.end(),initialHeight);
	distance.insert(distance.end(),0.0);

	logfile.open ("log.txt");

	logfile << "time   " << "DownRange   " << "Height   " << "HorizontalSpeed   " << "VerticalSpeed   "<<endl ;
	

	iteration =1;
}

void Dynamics::Init(double initialSpeed, double initialHeight, double angle)
{
	this->angle = (angle)* DEG2RAD;
	this->initialSpeed = initialSpeed;
	this->initialHeight = initialHeight;

	this->height.clear();
	this->distance.clear();
	this->idealHeight.clear();
	this->idealDistance.clear();
	this->time=0;
	this->idealTime=0;
	this->timeInterval=1;//large then 1

	this->currentHorizontalSpeed = cos(this->angle) * initialSpeed;
	this->currentVerticalSpeed = sin(this->angle) * initialSpeed;
	this->currentAirSpeed = initialSpeed;


	this->height.insert(height.end(),initialHeight);
	this->distance.insert(distance.end(),0.0);
	//log
	
	
	if (logfile.is_open())
		logfile << "Init #" << iteration << endl;
	else
	{
		logfile.open("log.txt", std::ofstream::app);
		logfile << "Init " << iteration << endl;
	}
		
	iteration++;	

}

void Dynamics::EstimateVelocityComponents(cv::Point2i currentPnt, cv::Point2i prevPnt,int currentBallRadius, int prevBallRadius, double &V, double &verticalAngle ,double &horizontalAngle)
{
	
	double dt12 = 0.033;  // difference time between succssive frames of the tracked object
	//assume we have 30 fps so, if we're tracking the object continiusly it will be: 

	double Vz,Vx,Vy;//velocity 
	double Vz2,Vx2,Vy2;//velocity squared
	double Vxy;//velocity in horizontal plane
	double factor1 = 5.0;
	Vz2 = factor1 * pow((float)(currentBallRadius - prevBallRadius) / dt12, 2);	//  velocity through principal axes
	Vx2 = (float)pow((float)(currentPnt.x - prevPnt.x) / dt12, 2);	// x component velocity (pixel/sec)
	Vy2 = (float)pow((float)(currentPnt.y - prevPnt.y) / dt12, 2);	// y component velocity (pixel/sec)
	Vxy = sqrt(Vx2 + Vy2) ;				// camera planar velocity, the only one we can measured
	
	Vx = sqrt(Vx2) ;
	Vy = sqrt(Vy2) ;
	Vz = sqrt(Vz2) ;
	V = sqrt(Vx2 + Vy2 + Vz) ;
	
	verticalAngle = RAD2DEG*acos(Vz/V);
	horizontalAngle = RAD2DEG*acos(Vx / Vz);
	return ;

}
//---------------------------CALCULATIONS------------------------------------


//Calculates air resistance for a given moment in time. This must be called in set intervals
//of time to continually update changing air resistance. 

//The drag force is proportional to the ball speed squared and is given by the formula
//F = Cd A d v2/2
void Dynamics::calculateAirResistance()
{
	airResistance = (1 / 2.0) * airDensity * pow(currentAirSpeed, 2) * area * dragCoefficient;
	// if v = 10 m/s then F = 0.114 N. 
	//If v = 20 m/s then F = 0.456 N.
	//If v = 30 m/s then F = 1.026 N.
}

//Speed is recalculated every PHYSICS_REFRESH_RATE milliseconds, and the calculated air resistance 
//can be modeled to act on the object for that period of time, allowing for the change 
//in speed to be calculated. 
void Dynamics::calculateCurrentSpeeds()
{
	calculateAirResistance();

	angle = atan2(currentVerticalSpeed, currentHorizontalSpeed);

	currentHorizontalSpeed -= ((airResistance * cos(angle)) / mass)
		* PHYSICS_REFRESH_RATE;
	currentVerticalSpeed -= (gravity + ((airResistance * sin(angle)) / mass))
		* PHYSICS_REFRESH_RATE;
	currentAirSpeed = hypot(currentHorizontalSpeed, currentVerticalSpeed);

}

//Using the calculated speeds we can just multiply by the time passed or PHYSICS_REFRESH_RATE
//to find where the object is at the new time. 
void Dynamics::calculateCurrentPositions()
{
	calculateCurrentSpeeds();

	height.insert(height.end(),height[timeInterval - 1] + (currentVerticalSpeed * PHYSICS_REFRESH_RATE));
	distance.insert(distance.end(),distance[timeInterval - 1] + (currentHorizontalSpeed * PHYSICS_REFRESH_RATE));

	time += PHYSICS_REFRESH_RATE;
}

bool Dynamics::onlyCalculateIdealPath()
{
	return (mass <= 0 || dragCoefficient <= 0 || area <= 0 || airDensity <= 0);
}

//Runs simulation to place appropriate values in array. 
void Dynamics::runSimulation()
{
	idealHeight.insert(idealHeight.end(), initialHeight);
	idealDistance.insert(idealDistance.end(),0.0);

	//timeInterval++;

	//Calculates ideal path. 
	do
	{

		idealTime += PHYSICS_REFRESH_RATE;

		idealHeight.insert(idealHeight.end(),(-(gravity / 2.0) * pow(idealTime, 2) + (initialSpeed * sin(angle)
			* idealTime)) + initialHeight);
		idealDistance.insert(idealDistance.end(),(idealDistance[timeInterval - 1] + (initialSpeed * cos(angle))
			* PHYSICS_REFRESH_RATE));

		timeInterval++;
		
	} while (idealHeight[timeInterval - 1] > idealHeight[0]);

	timeInterval = 1;

	if (onlyCalculateIdealPath())
	{
		height = idealHeight;
		distance = idealDistance;
		time = idealTime;

	}
	else
	{
		do
		{
			calculateCurrentPositions();
		
				
			logfile << timeInterval << "   " << distance[timeInterval] << "   " << height[timeInterval] << "   " << currentHorizontalSpeed << "   " <<currentVerticalSpeed<<  endl ; 

			timeInterval++;

		} while (height[timeInterval - 1] > height[0]);

	}

	idealTime -= 0.01;
	time -= 0.01;
	
	logfile.close();                                       // explicit close

}
