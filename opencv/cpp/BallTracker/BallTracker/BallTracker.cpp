#define _USE_MATH_DEFINES // to get M_PI
#include <iostream>
#include <stdexcept>
#include <stdint.h>
#include <cmath>
#include <functional>
#include "BallTracker.hpp"
#include "Utility.hpp"

#include <math.h>

using namespace std;
// Intialize CaptureType struct names
const char* CaptureType::names[] = {"camera", "video", "image"};

// List of extensions for image files
const std::string imageFileExtensions[] = {".png", ".jpg", ".jpeg", ".bmp", ".gif", ".dib",
    ".jp2", ".pbm", ".pgm", ".ppm", ".tiff", ".tif"};
std::vector<std::string> imageFileExtensionList(imageFileExtensions, end(imageFileExtensions));

//-------------------------------- Member functions -----------------------------
BallTracker::BallTracker(int deviceNum, bool debug, bool debugVerbose, bool trackBall, int camWidth,
    int camHeight, TrackingParameters *trackingParameters) {
	isRunning = false;
	interfaceIsInitialized = false;
	videoIsPaused = true;
	foundBall = false;
	foundBallAgain = false;
	ballCenter = cv::Point2i(0,0);
	ballRadius = 0;
    this->deviceNum = deviceNum;
    trackingEnabled = trackBall;
    captureProperties["width"] = (float)camWidth;
    captureProperties["height"] = (float)camHeight;
    this->trackingParameters = trackingParameters;
    this->trackingParameters->debug = debugVerbose;
    captureType = CaptureType::camera;
    this->debug = debug;
    this->debugVerbose = debugVerbose;
	//s.r.
	SetIntrinsicMatrix();

	ballVelocity = 0;
	ballVerticalAngle = 0;
	ballHorizontalAngle = 0;
	diffBetweenTracks=-1;
    // Open the capture
    cap = cv::VideoCapture(this->deviceNum);
    if (!cap.isOpened())
        throw std::runtime_error("failed to capture from device");

    // Set capture properties
    cap.set(cv::CAP_PROP_FRAME_WIDTH, camWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, camHeight);

    // Debug information
    if (debug) {
        std::cout << "Camera: device " << this->deviceNum << std::endl;
        std::cout << "Properties:" << std::endl << "\tWidth: " << captureProperties["width"] << std::endl
			<< "\tHeight: "<< captureProperties["height"] << std::endl;
        std::cout << "\tMax Width: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
        std::cout << "\tMax Height: " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
        /*
		std::cout << "\tMode: " << cap.get(cv::CAP_PROP_MODE) << std::endl;
        std::cout << "\tBrightness: " << cap.get(cv::CAP_PROP_BRIGHTNESS) << std::endl;
        std::cout << "\tContrast: " << cap.get(cv::CAP_PROP_CONTRAST) << std::endl;
        std::cout << "\tSaturation: " << cap.get(cv::CAP_PROP_SATURATION) << std::endl;
        std::cout << "\tHue: " << cap.get(cv::CAP_PROP_HUE) << std::endl;
        std::cout << "\tGain: " << cap.get(cv::CAP_PROP_GAIN) << std::endl;
        std::cout << "\tExposure: " << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;
		*/

        std::cout << "Ball tracker initialized." << std::endl;
    }
}

BallTracker::BallTracker(std::string fileName, bool debug, bool debugVerbose, bool trackBall,
    bool startVideoPaused, TrackingParameters *trackingParameters) {
	isRunning = false;
	frameNumber = 1;
<<<<<<< HEAD
=======
	frameNumberPrevTrack = 10000;//s.r.
>>>>>>> origin/sharons_dynamics
	interfaceIsInitialized = false;
	videoIsPaused = false;
	foundBall = false;
	foundBallAgain = false;
    this->fileName = fileName;
    trackingEnabled = trackBall;
    videoIsPaused = startVideoPaused;
    this->trackingParameters = trackingParameters;
    this->trackingParameters->debug = debugVerbose;
    captureType = CaptureType::video;
    this->debug = debug;
    this->debugVerbose = debugVerbose;
	SetIntrinsicMatrix();
	//s.r.

	ballVelocity = 0;
	ballVerticalAngle = 0;
	ballHorizontalAngle = 0;
	diffBetweenTracks=-1;
    // Check if we have an image file using the file extension
    std::string fileNameExt = fileName.substr(fileName.find_last_of(".") + 1);
    if (std::find(imageFileExtensionList.begin(), imageFileExtensionList.end(),
        fileNameExt) != imageFileExtensionList.end()) {
        captureType = CaptureType::image;
    }

    // Open the capture
    
	cap.open(fileName);
	
    if (!cap.isOpened())
		throw std::runtime_error("Failed to capture from file");
	//initial Dynamics object
	
	double initialSpeed = 10;
	double initialHeight = 1;
	double angle = 30;
	
	BallDynamics = new Dynamics(initialSpeed,initialHeight,angle);
	/*BallDynamics->runSimulation();///couple of tests
	BallDynamics->runSimulation();
	BallDynamics->Init(initialSpeed,initialHeight,angle);
	BallDynamics->runSimulation();*/

    // Get capture properties
    captureProperties["width"] = (float)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    captureProperties["height"] = (float)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    if (isUsingVideo()) {
        captureProperties["fps"] = (float)cap.get(cv::CAP_PROP_FPS);
        captureProperties["frameCount"] = (float)cap.get(cv::CAP_PROP_FRAME_COUNT);
        captureProperties["length"] = captureProperties["frameCount"] / captureProperties["fps"];
    }

    // Debug information
    if (debug) {
        std::cout << CaptureType::names[captureType] << " file: " << this->fileName << std::endl;
        std::cout << "Properties:" << std::endl
			<< "\tWidth: " << captureProperties["width"] << std::endl
			<< "\tHeight: " << captureProperties["height"] << std::endl;
        if (isUsingVideo()) {
            std::cout << "\tFPS: " << captureProperties["fps"] << std::endl;
            std::cout << "\tFrames: " << captureProperties["frameCount"] << std::endl;
            std::cout << "\tLength: " << captureProperties["length"] << std::endl;
        }
        std::cout << "Ball tracker initialized." << std::endl;
    }
}

BallTracker::~BallTracker() {
    if (cap.isOpened())
        cap.release();

    cv::destroyAllWindows();
}

bool BallTracker::isUsingVideo() {
    return captureType == CaptureType::video;
}

bool BallTracker::isUsingImage() {
    return captureType == CaptureType::image;
}

bool BallTracker::isUsingCamera() {
    return captureType == CaptureType::camera;
}

int BallTracker::getWidth() {
	return (int)captureProperties["width"];
}

int BallTracker::getHeight() {
	return (int)captureProperties["height"];
}

void BallTracker::SetIntrinsicMatrix()
{

	intrisicMat.create(3, 3, cv::DataType<double>::type);

	intrisicMat.at<double>(0, 0) = FOCAL_LENGTH;
	intrisicMat.at<double>(1, 0) = 0;
	intrisicMat.at<double>(2, 0) = 0;

	intrisicMat.at<double>(0, 1) = 0;
	intrisicMat.at<double>(1, 1) = FOCAL_LENGTH;
	intrisicMat.at<double>(2, 1) = 0;

	intrisicMat.at<double>(0, 2) = 0;
	intrisicMat.at<double>(1, 2) = 0;
	intrisicMat.at<double>(2, 2) = 1;

	rVec.create(3, 1, cv::DataType<double>::type);
	rVec.at<double>(0) = 0;
	rVec.at<double>(1) = 0;
	rVec.at<double>(2) = 0;

	tVec.create(3, 1, cv::DataType<double>::type);
	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = 0;

	distCoeffs.create(5, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = 0;
	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0;
	distCoeffs.at<double>(3) = 0;
	distCoeffs.at<double>(4) = 0;

	/*intrisicMat.at<double>(0, 0) = 1.6415318549788924e+003;
	intrisicMat.at<double>(1, 0) = 0;
	intrisicMat.at<double>(2, 0) = 0;

	intrisicMat.at<double>(0, 1) = 0;
	intrisicMat.at<double>(1, 1) = 1.7067753507885654e+003;
	intrisicMat.at<double>(2, 1) = 0;

	intrisicMat.at<double>(0, 2) = 5.3262822453148601e+002;
	intrisicMat.at<double>(1, 2) = 3.8095355839052968e+002;
	intrisicMat.at<double>(2, 2) = 1;

	rVec.create(3, 1, cv::DataType<double>::type);
	rVec.at<double>(0) = -3.9277902400761393e-002;
	rVec.at<double>(1) = 3.7803824407602084e-002;
	rVec.at<double>(2) = 2.6445674487856268e-002;

	tVec.create(3, 1, cv::DataType<double>::type);
	tVec.at<double>(0) = 2.1158489381208221e+000;
	tVec.at<double>(1) = -7.6847683212704716e+000;
	tVec.at<double>(2) = 2.6169795190294256e+001;

	distCoeffs.create(5, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = -7.9134632415085826e-001;
	distCoeffs.at<double>(1) = 1.5623584435644169e+000;
	distCoeffs.at<double>(2) = -3.3916502741726508e-002;
	distCoeffs.at<double>(3) = -1.3921577146136694e-002;
	distCoeffs.at<double>(4) = 1.1430734623697941e+002;*/



}
void BallTracker::run() {
	if (! cap.isOpened())
		return;

	if (! interfaceIsInitialized)
		initializeInterface();

	frameNumber = 1;
	bool isRunning = true;
	bool wantNextFrame = true; // want a new frame
	bool haveNewFrame = false; // new frame read
	int result;
	cv::Mat frame(getHeight(),getWidth(), CV_8UC3), 
			frameFiltered(getHeight(),getWidth(), CV_8UC3), 
			frameBall(getHeight(), getWidth(), CV_8UC3);

	// Only load images once
	if(isUsingImage()) {
		result = cap.read(frame);
		haveNewFrame = true;
		cap.release();
	}
	
	backgroundSubtractor = cv::createBackgroundSubtractorMOG2();

	// Main loop
	while (isRunning) {
		// Read the frame if not using an single image
		if (!isUsingImage()) {
			if (videoIsPaused && !wantNextFrame) {
				// do nothing
			}
			else {
				result = cap.read(frame);
				wantNextFrame = false;
				haveNewFrame = true;
			}
		}
		frame.copyTo(frameBall);

		// Process frame
		filterFrame(frame, frameFiltered);
		if (trackingEnabled) {
			foundBall = detectBall(frameFiltered, ballCenter, ballRadius);

			if (foundBall) {
				drawBall(frameBall,ballCenter,ballRadius);
<<<<<<< HEAD
=======

				//s.r.
				diffBetweenTracks = frameNumber-frameNumberPrevTrack;
				if (foundBall && (diffBetweenTracks)>0)//check time history between tracks
				{
				//work with couple of tracks

					BallDynamics->EstimateVelocityComponents(ballCenter, ballCenterPrevTrack,ballRadius, ballRadiusPrevTrack, ballVelocity, ballVerticalAngle ,ballHorizontalAngle,diffBetweenTracks);
					BallDynamics->Init(ballVelocity, ballCenter.y, ballVerticalAngle, ballRadius);

					//test Only
					//BallDynamics->SetIdealPath();
					
					if(!_isnan(ballVerticalAngle))
					{
						BallDynamics->runSimulation();
						// Read 3D points
						std::vector<cv::Point3d> objectPoints = Generate3DPoints();

						drawPredictedTrajectory(objectPoints, ballCenter.x, ballCenter.y, frameBall);

					}
					else
						cout << "vertical angle estimation out of limit" << endl;
				}
				else
				{
					frameNumberPrevTrack = frameNumber;
					ballCenterPrevTrack = ballCenter;
					ballRadiusPrevTrack = ballRadius;
				}
				

>>>>>>> origin/sharons_dynamics
			}
		}

		// Frame info
		if (isUsingVideo()) {
			double textScale = 0.8f;
			int textThickness = 1;
			cv::Scalar textColor(255,160,160);
			char *pausedStr = "";
			if (videoIsPaused)
				pausedStr = " (paused)";
			sprintf(tempBuf, "Frame %u of %u%s.", frameNumber, (int)captureProperties["frameCount"], pausedStr);
			cv::putText(frameBall, std::string(tempBuf),cv::Point(0,(int)captureProperties["height"]-10),
				cv::FONT_HERSHEY_PLAIN, textScale, textColor, textThickness);
		}

		// Debug info
		if (debug && (trackingParameters->wasChanged() || haveNewFrame)) {
			tempBuf[0] = '.'; tempBuf[1] = '\0';

			if (isUsingVideo())
				sprintf(tempBuf, " at frame %u of %u.", frameNumber, (unsigned int)captureProperties["frameCount"]);

			if (foundBall)
				std::cout << "Found ball at " << ballCenter.x << ", " << ballCenter.y << " with radius "
					<< ballRadius << tempBuf << std::endl;
			else
				std::cout << "Did not find ball" << tempBuf << std::endl;
		}

		// Show the frames
		cv::imshow("Camera", frameBall);
		cv::imshow("Filtered", frameFiltered);

		// Get key presses
		int key = cv::waitKey(1) & 0xFF;
		if (key == 'q')
			break;
		else if (key == 'n')
			wantNextFrame = true;
		else if (key == 'p')
			trackingParameters->printParameters();
		else if (key == 13)
			videoIsPaused = videoIsPaused ^ 1;

		// Advancing to next frame
		if (isUsingVideo() && !videoIsPaused)
			wantNextFrame = true;
		if (!isUsingImage() && wantNextFrame)
			frameNumber++;

		// Video loops
		if (isUsingVideo() && frameNumber >= (int)captureProperties["frameCount"]) {
			frameNumber = 1;
			cap.release();
			cap = cv::VideoCapture(fileName);
		}
		haveNewFrame = false;
	} // while (isRunning())

	if (debug) {
		trackingParameters->printParameters();
		std::cout << "Ball tracker finished." << std::endl;
	}
}

void BallTracker::initializeInterface() {
	if (interfaceIsInitialized)
		return;

	// Create windows
    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Filtered", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Controls", cv::WINDOW_NORMAL);

	// Filter window trackbars
	createTrackbar("hueLower", "Controls", FILTER_HSV_MAX);
	createTrackbar("hueUpper", "Controls", FILTER_HSV_MAX);
	createTrackbar("satLower", "Controls", FILTER_HSV_MAX);
	createTrackbar("valLower", "Controls", FILTER_HSV_MAX);
	createTrackbar("filterBackgroundOn", "Controls", 1);
	createTrackbar("filterIterations", "Controls",  FILTER_ITERATIONS_MAX);
	createTrackbar("filterKernel", "Controls", FILTER_KERNEL_MAX); // rect = 0, ellipse = 1, cross = 2
	createTrackbar("filterKernelSize", "Controls", FILTER_KERNEL_SIZE_MAX);
	createTrackbar("filterBlurOn", "Controls", 1);
	createTrackbar("filterSigma", "Controls", FILTER_SIGMA_MAX);

	// Hough detection trackbars
	createTrackbar("houghOn", "Controls", 1);
	createTrackbar("houghThreshUpper", "Controls", HOUGH_THRESH_MAX);
	createTrackbar("houghThreshLower", "Controls", HOUGH_THRESH_MAX);
	createTrackbar("houghMinRadius", "Controls", HOUGH_RADIUS_MAX);
	createTrackbar("houghMaxRadius", "Controls", HOUGH_RADIUS_MAX);

	// Contour detection trackbars
	createTrackbar("contourMaxArea", "Controls", CONTOUR_MAX_AREA_MAX);
	createTrackbar("contourMinArea", "Controls", CONTOUR_MIN_AREA_MAX);
}

void BallTracker::createTrackbar(const char* parameterName, const char* windowName, int maxValue) {
	tempInt = trackingParameters->getParameter(parameterName);
	cv::createTrackbar(parameterName, windowName, &tempInt, maxValue, sliderCallback,
		createSliderCallbackData(trackingParameters, parameterName));
}

void BallTracker::filterFrame(cv::Mat &src, cv::Mat &dst) {
	// Filter with background subtraction
	cv::Mat frameGray(src.size(), src.type());
	cv::cvtColor(src,frameGray, cv::COLOR_BGR2BGRA);
	backgroundSubtractor->apply(frameGray,backgroundMask,0);

	// Filter with HSV thresholds
	cv::Mat frameHsv(src.size(), src.type());
	cv::cvtColor(src,frameHsv, cv::COLOR_BGR2HSV);
	cv::Scalar hsvRangeLower = cv::Scalar(trackingParameters->getParameter("hueLower"),
		trackingParameters->getParameter("satLower"),
		trackingParameters->getParameter("valLower"), 0);
	cv::Scalar hsvRangeUpper = cv::Scalar(trackingParameters->getParameter("hueUpper"),
		trackingParameters->getParameter("satUpper"),
		trackingParameters->getParameter("valUpper"), 0);

	cv::inRange(frameHsv, hsvRangeLower, hsvRangeUpper, dst);

	// Combine with background filtering if enabled
	if (trackingParameters->getParameter("filterBackgroundOn"))
		dst = dst & backgroundMask;

	// Filter image with dilation, guassian blur, and erosion
	int iterations = trackingParameters->getParameter("filterIterations");
	int kernelSize = trackingParameters->getParameter("filterKernelSize");
	cv::Mat kernel = cv::getStructuringElement(trackingParameters->getParameter("filterKernel"), cv::Size(kernelSize,kernelSize));
	// Dilate
	for (uint8_t i = 0; i < iterations; i++)
		cv::dilate(dst, dst, kernel); 

	// Gaussian blur if enabled
	if (trackingParameters->getParameter("filterBlurOn"))
		cv::GaussianBlur(dst, dst, cv::Size(0,0), std::max(trackingParameters->getParameter("filterSigma"),1),
			std::max(trackingParameters->getParameter("filterSigma"),1));

	// Erode
	for (uint8_t i = 0; i < iterations; i++)
		cv::erode(dst, dst, kernel); 
}

bool BallTracker::detectBall(cv::Mat &frame, cv::Point2i &center, int &radius) {
	bool found = false;
	radius = 0;
	center.x = 0;
	center.y = 0;

	if (trackingParameters->getParameter("houghOn")) {
		std::vector<cv::Vec3f> circles;
		HoughCircles(frame, circles, cv::HOUGH_GRADIENT, 1,
			trackingParameters->getParameter("houghMinDist"), std::max(trackingParameters->getParameter("houghThreshUpper"),1),
			std::max(trackingParameters->getParameter("houghThreshLower"),1), trackingParameters->getParameter("houghMinRadius"),
			trackingParameters->getParameter("houghMaxRadius"));

		// Find the largest circle
		float maxRadius = 0.0;
		float x = 0;
		float y = 0;

		for (size_t i = 0; i < circles.size(); i++) {
			x = circles[i][0];
			y = circles[i][1];
			if (circles[i][2] > maxRadius) {
				maxRadius = circles[i][2];
				found = true;
				center.x = int(x + 0.5f);
				center.y = int(y + 0.5f);
				radius = int(maxRadius + 0.5f);
			}
		}
	} // hough circle detection
	else {
		// Contour detection
		cv::Mat temp;
		frame.copyTo(temp);
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;

		cv::findContours(temp,contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

		// Look for largest moment
		double maxArea = 0.0;
		float x = 0.0, y = 0.0;
		int numObjects = hierarchy.size();
		if (numObjects > DEFAULT_CONTOUR_MAX_OBJECTS) {
			if (debug)
				std::cout << "Contour detection found too many objects!" << std::endl;
		}
		else if (numObjects > 0) {
			// Use hierarchy to determine next 
			for (int i = 0; i >= 0; i = hierarchy[i][0]) {
			//for (int i = 0; i < numObjects; i++) {
				cv::Moments moment = cv::moments((cv::Mat)contours[i]);
				double objectArea = moment.m00;
				double objectRadius = sqrt(objectArea/M_PI);
				if (objectArea > trackingParameters->getParameter("contourMinArea")
					&& objectArea < trackingParameters->getParameter("contourMaxArea")
					&& objectArea > maxArea) {
						found = true;
						maxArea = objectArea;
						center.x = (int)(moment.m10/objectArea);
						center.y = (int)(moment.m01/objectArea);
						radius = (int)objectRadius;
				}

			}
		}
	} // contour detection

	return found;
}


void BallTracker::drawBall(cv::Mat &frame, cv::Point2i center, int outerRadius, int innerRadius,
	const cv::Scalar& innerColor, const cv::Scalar& outerColor, const cv::Scalar& crosshairColor) {
	// Draw circles
	cv::circle(frame,center,innerRadius,innerColor,-1);
	cv::circle(frame,center,outerRadius,outerColor,1);

	// Draw crosshairs
	int crosshairSize = (int)((float)outerRadius*2.2f);
	int crosshairThickness = 1;
	int crosshairOffset = crosshairSize/2;
	cv::line(frame,cv::Point(center.x - crosshairOffset,center.y),
		cv::Point(center.x + crosshairOffset,center.y),
		crosshairColor, crosshairThickness);
	cv::line(frame, cv::Point(center.x, center.y - crosshairOffset),
		cv::Point(center.x, center.y + crosshairOffset),
		crosshairColor, crosshairThickness);

	// Add text
	char buf[250];
	double textScale = 0.8f;
	int textThickness = 1;
	cv::Scalar textColor(0,255,0);
	sprintf(buf,"(x: %u, y:%u), %u px",center.x, center.y, outerRadius);
	cv::putText(frame, std::string(buf), cv::Point(center.x + outerRadius, center.y + outerRadius),
		cv::FONT_HERSHEY_PLAIN, textScale, textColor, textThickness);
<<<<<<< HEAD
=======
}



std::vector<cv::Point3d> BallTracker::Generate3DPoints()
{
	std::vector<cv::Point3d> points;

	double x, y, z;
	double y0 = BallDynamics->height[0];
	double z0 = BallDynamics->x[0];
	int sz = BallDynamics->x.size();
	for (unsigned int i = 0; i < sz; ++i)
	{
		
		
		if (BallDynamics->height[i] > y0)
		{
			y = y0 - fabs(BallDynamics->height[i] - y0);
		}
		else
			y = y0 + fabs(BallDynamics->height[i] - y0);


		if (BallDynamics->x[i] > z0)
		{
			z = z0 - fabs(BallDynamics->x[i] - z0);
		}
		else
			z = z0 + fabs(BallDynamics->x[i] - z0);
		
		
		x = BallDynamics->z[i];// transfomation with camera and trajectory coordinate system
		z = BallDynamics->x[i];// transfomation with camera and trajectory coordinate system

		points.push_back(cv::Point3d(x, y, z));
	}

	return points;
}

void BallTracker::drawPredictedTrajectory(std::vector<cv::Point3d> points, int x, int y, Mat &frame)
{

	//cv::Mat imageToDraw; //this is your image to draw, don't forget to load it
	//std::vector<cv::Point> pointsInLast20Frames; //fill this vector with points, they should be ordered
	cv::Scalar color(0, 0, 255); //red
	std::vector<cv::Point2d> imagePoints;

	cv::Mat invert_intrisicMat(3, 3, cv::DataType<double>::type); // Intrisic matrix
	//cv::invert(intrisicMat, invert_intrisicMat);

	cv::projectPoints(points, rVec, tVec, intrisicMat, distCoeffs, imagePoints);
	
	int sz = points.size() - 1;
	for (int i = 0; i < sz; ++i)
	{
		int px= this->BallDynamics->EstimateImageCoord( points[i].x,points[i].z);
		int py= this->BallDynamics->EstimateImageCoord( points[i].y,points[i].z);

		imagePoints[i].x = points[i].x / points[i].z * FOCAL_LENGTH;
		imagePoints[i].y = points[i].y / points[i].z * FOCAL_LENGTH;

		imagePoints[i+1].x = points[i+1].x / points[i+1].z * FOCAL_LENGTH;
		imagePoints[i+1].y = points[i+1].y / points[i+1].z * FOCAL_LENGTH;

		cv::line(frame, imagePoints[i], imagePoints[i+1], color);
		
	}

	//cv::line(frame, Point(x1, y1),Point(x2, y2), color);

>>>>>>> origin/sharons_dynamics
}

/*----------------------- Non-member Functions ------------------------*/
void sliderCallback(int val, void *userData) {
	SliderCallbackData data = (SliderCallbackData)userData;
	std::string parameterName(data->parameterName);
	data->obj->changeParameter(val, parameterName);
}

SliderCallbackData createSliderCallbackData(TrackingParameters *obj, const char *name) {
	SliderCallbackData data = new _sliderCallbackData();
	data->obj = obj;
	data->parameterName = name;
	return data;
}