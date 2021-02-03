// CameraTuner.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
//#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
//#include <sys/time.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>

//#include <unistd.h>
#include <fcntl.h>
#include <string.h>
//#include <sys/ioctl.h>

//#include "Arduino.h"
//#include "UdpSocketLinux.h"
#include "SteamWorks.h"

// Mat and Erin
// use a real Makefile
// use match time in the imwrite filename ????
// find Acadia enclosure that fits with shield ????
// find ear piece for driver station ????
// how do I fix the lag between the Acadia and the laptop ???? Will H.264 help?
// remember that the boiler is on the left for red alliance and the boiler is on the right for Blue 
// read manual updates on Tuesdays and Thursday
// place ntpdate 10.58.58.3 in /etc/rc.local
// place the watchdog in /etc/init.d/
// clean Logs folders often
// backup to flash drives
// add try: to several functions
// use -lpthread in Makefile

using namespace std;
using namespace cv;

//VideoCapture *front_capture;
//VideoCapture *back_capture;

//class UdpSocket Net;
//class UdpSocket ImageNet;

//const int RING_LIGHT_PIN1 = GPIO7;
//const int RING_LIGHT_PIN2 = GPIO13;
//const int GO_LEFT_PIN = GPIO6;
//const int GO_RIGHT_PIN = GPIO9;
//const int CENTERED_PIN = GPIO10;

//vector<int> compression;
//int debug_mode = 1;
//bool useFileImage = false;
//bool receivedInitMessage = false;
//int camera = ONE_CAMERA; // TEC

int image_request_type = GRAY_TYPE;
int image_request_crop = CROPPED;
int image_request_freq = 5; // period between transmits, 0 means never send, 255 means always

int currentState = LEFT_DEADBAND_STATE;
//int hopper_side = 1; // 0-left, 1-right
//int send_error_mode = 1;
int target_mode = GEAR_ON_GROUND_TARGET; // GEAR_PIN_TARGET; // HIGH_FUEL_TARGET;
int track_error_x = 0; // pixels
int range_to_target = 0; // inches
//int cg_x = 0;
//int cg_y = 0;

int crop_top = 10;
int crop_bottom = screen_height - 10;

int MinH = 0;
int MaxH = 179;//180?
int MinS = 0;
int MaxS = 255;//256
int MinV = 0;
int MaxV = 255;
int RatioMin = 20;
int RatioMax = 142;
int ContourAreaMin = 10;
int ContourAreaMax = 255;
int Threshold = 127;

// upper goal variables
Scalar minHSVs[TOTAL_TARGETS];
Scalar maxHSVs[TOTAL_TARGETS];
// high, low, ground fuel, hopper, rope, gear_pin, feeder, ground gear
int RatioMins[TOTAL_TARGETS] = { 120, 20, 20, 120, 20, 26, 120, 24 };
int RatioMaxs[TOTAL_TARGETS] = { 240, 40, 240, 140, 100, 141, 240, 458 };
int ContourAreaMins[TOTAL_TARGETS] = { 90, 30, 20, 200, 20, 1100, 300, 135 };
int ContourAreaMaxs[TOTAL_TARGETS] = { 255, 255, 255, 400, 255, 2200, 700, 1757 };
//int GimbalAngles[TOTAL_TARGETS] = { 9, 9, 9, 9, 9, 9, 9, 9 };
int CropTops[TOTAL_TARGETS] = { 200, 9, 9, 9, 9, 252, 9, 249 };
int CropBottoms[TOTAL_TARGETS] = { 400, 400, 400, 400, 400, 395, 400, 400 };

double confidence_level = 0.0; // 0-100
int numContoursFound = 0;
//bool imagesLocked = true;

int alliance_color = RED;
int match_time = 0; // int seconds
double dt = 0.0;
//char IP[32]; // IP address for UDP

Mat rawImage;
Mat HSVImage;
Mat HSVImageFiltered;
Mat binaryImage;
Mat croppedRawImage;
Mat grayImage;

//pthread_t image_thread;
//pthread_t udp_thread;
//void SendImage();

/*
void findIP()
{
	char returnData[64];
	FILE *fp = popen("/sbin/ifconfig eth0", "r");

	char *fgets_ret;
	fgets_ret = fgets(returnData, 64, fp);
	fgets_ret = fgets(returnData, 64, fp);

	int cnt = 0;
	int ndx = 10;
	while (returnData[ndx] != ':') {
		if (ndx > 45) {
			printf("Could not find IP address\n");
			exit(0);
		}
		ndx++;
	}

	//printf("ndx %d\n", ndx );
	int numPeriods = 0;
	for (int i = ndx + 1; i < 30; i++) {
		if (returnData[i] == '.') numPeriods++;

		if (numPeriods == 3)
			break;
		else
			IP[cnt++] = returnData[i];
	}
	IP[cnt++] = '.';
	IP[cnt++] = '2';
	IP[cnt++] = '5';
	IP[cnt++] = '5';
	IP[cnt++] = '\0';
	printf("%s\n", IP);
	fflush(stdout);
}
*/

void initializeNewTargetMode()
{
	setTrackbarPos("MinH", "MainWindow", minHSVs[target_mode][0]);
	setTrackbarPos("MinS", "MainWindow", minHSVs[target_mode][1]);
	setTrackbarPos("MinV", "MainWindow", minHSVs[target_mode][2]);
	setTrackbarPos("MaxH", "MainWindow", maxHSVs[target_mode][0]);
	setTrackbarPos("MaxS", "MainWindow", maxHSVs[target_mode][1]);
	setTrackbarPos("MaxV", "MainWindow", maxHSVs[target_mode][2]);

	setTrackbarPos("Ratio Min", "MainWindow", RatioMins[target_mode]);
	setTrackbarPos("Ratio Max", "MainWindow", RatioMaxs[target_mode]);

	setTrackbarPos("Area Min", "MainWindow", ContourAreaMins[target_mode]);
	setTrackbarPos("Area Max", "MainWindow", ContourAreaMaxs[target_mode]);
}

void loadScrollBars()
{
	crop_top = CropTops[target_mode];
	crop_bottom = CropBottoms[target_mode];
	MinH = minHSVs[target_mode][0];
	MinS = minHSVs[target_mode][1];
	MinV = minHSVs[target_mode][2];
	MaxH = maxHSVs[target_mode][0];
	MaxS = maxHSVs[target_mode][1];
	MaxV = maxHSVs[target_mode][2];
	RatioMin = RatioMins[target_mode];
	RatioMax = RatioMaxs[target_mode];
	ContourAreaMin = ContourAreaMins[target_mode];
	ContourAreaMax = ContourAreaMaxs[target_mode];

	initializeNewTargetMode();
}

int handle_HIGH_FUEL_TARGET_TrackError(Rect boundRect)
{
	track_error_x = boundRect.x + (boundRect.width / 2);
	//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height / boundRect.width;

	printf("ratio = %f\n", ratio);
	if ((ratio > 0.01*RatioMins[target_mode]) &&
		(ratio < 0.01*RatioMaxs[target_mode])) {
		numContoursFound += 1;
		confidence_level += 0.5; //
		return 1;
	}
	else if ((ratio > 0.1) && (ratio < 0.2)) {
		numContoursFound += 1;
		confidence_level += 0.5; //
		// use a 5th order poly to estimate range ????
		double Gain = 1.0;
		range_to_target = (int)(144.0 + Gain*boundRect.width); // inches
		return 1;
	}
	return 0;

}

int handle_LOW_FUEL_TARGET_TrackError(Rect boundRect)
{
	static double sum_x = 0;
	track_error_x = 0;
	if (numContoursFound == 0) sum_x = 0;
	//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height / boundRect.width;

	printf("ratio = %f\n", ratio);
	// the lower goal contours are small and 8.75 tall x 3" wide = rati of 2.9
	if ((ratio > 0.01*RatioMins[target_mode]) &&
		(ratio < 0.01*RatioMaxs[target_mode])) {
		numContoursFound += 1;
		confidence_level += 0.125; //
		// get the geometric center
		sum_x += boundRect.x;
		track_error_x = (double)(1.0*sum_x / numContoursFound) + boundRect.width;
		// use a 5th order poly to estimate range ????
		double Gain = 1.0;
		range_to_target = (int)(44.0 + Gain*boundRect.width); // inches
		return 1;
	}
	return 0;
}

int handle_FUEL_ON_GROUND_TARGET_TrackError(Rect boundRect)
{
	static int sum_x = 0;
	track_error_x = 0;
	if (numContoursFound == 0) sum_x = 0;
	//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height / boundRect.width;

	printf("ratio = %f\n", ratio);
	if ((ratio > 0.01*RatioMins[target_mode]) &&
		(ratio < 0.01*RatioMaxs[target_mode])) {
		numContoursFound += 1;
		confidence_level += 1.0; //
		sum_x += boundRect.x;
		track_error_x = (double)(1.0*sum_x / numContoursFound) + boundRect.width;
		return 1;
	}
	return 0;

}

int handle_HOPPER_TARGET_TrackError(Rect boundRect)
{
	static int center_x = 0;
	track_error_x = 0;
	// this target should be tracking the left or right hopper. use hopper_side variable ????
	//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height / boundRect.width;

	printf("ratio = %f\n", ratio);
	if (numContoursFound > 0) {
		if (boundRect.x < center_x) {
			numContoursFound += 1;
			track_error_x = boundRect.x + (boundRect.width / 2) - screen_widthO2;
		}
		return 1;
	}
	// hopper_side 0 means track the left hopper ????
	// hopper is 26.5x23.25 = rati oof 1.14
	else if ((ratio > 0.01*RatioMins[target_mode]) &&
		(ratio < 0.01*RatioMaxs[target_mode])) {
		numContoursFound += 1;
		confidence_level += 1.0; //
		track_error_x = boundRect.x + (boundRect.width / 2) - screen_widthO2;
		// use a 5th order poly to estimate range ????
		double Gain = 1.0;
		range_to_target = (int)(144.0 + Gain*boundRect.width); // inches
		return 1;
	}
	return 0;
}

int handle_ROPE_TARGET_TrackError(Rect boundRect)
{
	// this rope tracker found 3 long contours. Just average all of the contours
	track_error_x = 0;

	//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height / boundRect.width;
	static double sum_x = 0.0;
	printf("rope ratio = %f\n", ratio);
	if ((ratio > 0.01*RatioMins[target_mode]) &&
		(ratio < 0.01*RatioMaxs[target_mode])) {
		numContoursFound += 1;
		confidence_level = 1.0; //
		sum_x += boundRect.x + (boundRect.width / 2);
		track_error_x = sum_x / numContoursFound;
		return 1;
	}
	return 0;
}

int handle_GEAR_ON_GROUND_TARGET_TrackError(Rect boundRect)
{
	static int sum_x = 0;
	track_error_x = 0;

	if (numContoursFound == 0) sum_x = 0;

	//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height / boundRect.width;
	//printf("ratio = %f\n", ratio  );
	if ((ratio > 0.01*RatioMins[target_mode]) &&
		(ratio < 0.01*RatioMaxs[target_mode])) {
		numContoursFound += 1;
		confidence_level += 0.5; //
		sum_x += boundRect.x + boundRect.width / 2;
		//printf("numContoursFound = %d\n", numContoursFound );
		//if( numContoursFound == 2 ) {
		track_error_x = sum_x / numContoursFound;

		printf("track_error_x = %d\n", track_error_x);
		//}
		// use a 5th order poly to estimate range ????
		double Gain = 1.0;
		range_to_target = (int)(144.0 + Gain*boundRect.width); // inches
		return 1;
	}
	return 0;
}

int handle_GEAR_PIN_TARGET_TrackError(Rect boundRect)
{
	static int sum_x = 0;
	track_error_x = 0;

	if (numContoursFound == 0) sum_x = 0;

	//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height / boundRect.width;
	//printf("ratio = %f\n", ratio  );
	if ((ratio > 0.01*RatioMins[target_mode]) &&
		(ratio < 0.01*RatioMaxs[target_mode])) {
		numContoursFound += 1;
		confidence_level += 0.5; //
		sum_x += boundRect.x;
		//printf("numContoursFound = %d\n", numContoursFound );
		//if( numContoursFound == 2 ) {
		track_error_x = sum_x / numContoursFound;

		printf("track_error_x = %d\n", track_error_x);
		//}
		// use a 5th order poly to estimate range ????
		double Gain = 1.0;
		range_to_target = (int)(144.0 + Gain*boundRect.width); // inches
		return 1;
	}
	return 0;
}

int handle_FEEDER_TARGET_TrackError(Rect boundRect)
{
	// have the feeder guy hold up a large reflective triangle
	track_error_x = 0;

	//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the left or right feeder or both
	double ratio = 1.0*boundRect.height / boundRect.width;

	printf("ratio = %f\n", ratio);
	// feeder opening is 25" wide by 7.5" tall = 0.3
	if ((ratio > 0.01*RatioMins[target_mode]) &&
		(ratio < 0.01*RatioMaxs[target_mode])) {
		numContoursFound += 1;
		confidence_level = 1.0; //
		track_error_x = boundRect.x + (boundRect.width / 2) - screen_widthO2;
		// use a 5th order poly to estimate range ????
		double Gain = 1.0;
		range_to_target = (int)(144.0 + Gain*boundRect.width); // inches
		return 1;
	}
	return 0;
}

void Track()
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	//printf("targetMode=%d: MaxHSV %f %f %f\n", target_mode, 
	//maxHSVs[target_mode][0], maxHSVs[target_mode][1], maxHSVs[target_mode][2]);
	//printf("targetMode=%d: MinHSV %f %f %f\n", target_mode, 
	//minHSVs[target_mode][0], minHSVs[target_mode][1], minHSVs[target_mode][2]);
	//printf("crop=%d %d\n", CropTops[target_mode], CropBottoms[target_mode] );
	cvtColor(croppedRawImage, HSVImage, CV_BGR2HSV);

	// filter Hue values outside of this range
	inRange(HSVImage, minHSVs[target_mode], maxHSVs[target_mode], HSVImageFiltered);

	// convert the filtered image to a binary format for contouring
	threshold(HSVImageFiltered, binaryImage, Threshold, 255, 0);

	// save the binaryImage because the findContours() will destroy it
	Mat tempBinaryImage;
	binaryImage.copyTo(tempBinaryImage);
	findContours(tempBinaryImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

	confidence_level = 0.0;
	numContoursFound = 0;
	range_to_target = 0; // inches

	//printf("Contour size=%d\n", contours.size() );
	if (contours.size()>0) {
		vector<Point> contours_poly(contours.size());

		// Approximate contours to polygons + get bounding rects and circles
		for (int cnt = 0; cnt<contours.size(); ++cnt) {
			approxPolyDP(Mat(contours[cnt]), contours_poly, 3, true);
			int contour_area = contourArea(contours[cnt]);
			//printf( "contour_area = %d\n", contour_area );

			if ((contour_area > ContourAreaMins[target_mode]) &&
				(contour_area < ContourAreaMaxs[target_mode])) { // estimate
				Rect boundRect = boundingRect(Mat(contours_poly));

				printf("in contour_area = %d\n", contour_area);
				printf("in boundRect = x=%d, width=%d\n", boundRect.x, boundRect.width);

				int good_track = 0;
				if (target_mode == HIGH_FUEL_TARGET)
					good_track = handle_HIGH_FUEL_TARGET_TrackError(boundRect);
				else if (target_mode == LOW_FUEL_TARGET)
					good_track = handle_LOW_FUEL_TARGET_TrackError(boundRect);
				else if (target_mode == FUEL_ON_GROUND_TARGET)
					good_track = handle_FUEL_ON_GROUND_TARGET_TrackError(boundRect);
				else if (target_mode == HOPPER_TARGET)
					good_track = handle_HOPPER_TARGET_TrackError(boundRect);
				else if (target_mode == ROPE_TARGET)
					good_track = handle_ROPE_TARGET_TrackError(boundRect);
				else if (target_mode == GEAR_PIN_TARGET)
					good_track = handle_GEAR_PIN_TARGET_TrackError(boundRect);
				else if (target_mode == FEEDER_TARGET)
					good_track = handle_FEEDER_TARGET_TrackError(boundRect);
				else if (target_mode == GEAR_ON_GROUND_TARGET)
					good_track = handle_GEAR_ON_GROUND_TARGET_TrackError(boundRect);

				//printf("error=%d\n", track_error_x );
				if (abs(track_error_x) < DeadBand) {
					//printf( "Center %d\n", track_error_x);
					currentState = CENTERED_DEADBAND_STATE;
				}
				else if (track_error_x< -DeadBand) {
					//printf( "LEFT %d\n", track_error_x);
					currentState = LEFT_DEADBAND_STATE;
				}
				else {
					//printf( "RIGHT %d\n", track_error_x);
					currentState = RIGHT_DEADBAND_STATE;
				}//if
				//ControlDirectionLEDs();
				track_error_x -= screen_widthO2;

				//if ( debug_mode == 1)
				if (good_track == 0) {
					rectangle(croppedRawImage, boundRect, Scalar(0, 255, 0), 1);
				}
				else
					rectangle(croppedRawImage, boundRect, Scalar(255, 255, 255), 2);
				/*
				if( debug_mode == 2 ) {
				printf("cnt=%d\n", cnt );
				drawContours( croppedRawImage, contours_poly,
				cnt, Scalar (0,255,0), 2 ); // last is thickness
				}
				*/
			} // if contour_area
		}//for cnt
	} // if any cnt
	if (confidence_level > 1.0) confidence_level = 1.0;

	if (1) {
		std::stringstream errorStr;
		errorStr << track_error_x;
		putText(croppedRawImage,
			"Error: " + errorStr.str() + ",",
			Point(20, 30), FONT_HERSHEY_COMPLEX, 0.5, // font size is last
			Scalar(0, 0, 0), 1, CV_AA);

		std::stringstream rangeStr;
		rangeStr << range_to_target;
		putText(croppedRawImage,
			"Range: " + rangeStr.str() + ",",
			Point(150, 30), FONT_HERSHEY_COMPLEX, 0.5, // font size is last
			Scalar(0, 0, 0), 1, CV_AA);

		std::stringstream confidenceStr;
		confidenceStr << (int)(confidence_level*100.0);
		putText(croppedRawImage,
			"Confidence: " + confidenceStr.str() + ",",
			Point(260, 30), FONT_HERSHEY_COMPLEX, 0.5, // font size is last
			Scalar(0, 0, 0), 1, CV_AA);

		std::stringstream modeStr;
		modeStr << target_mode_names[target_mode];
		putText(croppedRawImage,
			"Mode: " + modeStr.str() + ",",
			Point(410, 30), FONT_HERSHEY_COMPLEX, 0.5, // font size is last
			Scalar(0, 0, 0), 1, CV_AA);
	}
}

void on_mouse(int event, int x, int y, int flags, void* userdata)
{
	static int start_x = 0, start_y = 0;
	static int end_x = 0, end_y = 0;
	bool endComplete = false;
	//const int channels[] = { 0, 1, 2 };

	if (event == EVENT_LBUTTONDOWN) {
		printf("Start Mouse Position: %d %d\n", x, y);
		start_x = x;
		start_y = y;
	}
	else if (event == EVENT_LBUTTONUP) {
		printf("End Mouse Position: %d, %d\n", x, y);
		end_x = x;
		end_y = y;
		endComplete = true;
	}
	// now go into the imag array and find the histogram of the area
	if (endComplete) {
		endComplete = false;

		// get the subset image from the numpy array
		const Rect roi(start_x, start_y, end_x - start_x + 1,
			end_y - start_y + 1);
		printf("%d %d %d %d\n", start_x, start_y, end_x, end_y);
		printf("area=%d\n", (end_x - start_x)*(end_y - start_y));

		// this feature is only needed for HSV values
		Mat subset;
		Mat image_roi = HSVImage(roi);
		image_roi.copyTo(subset);

		vector<Mat> subset_split(3);
		split(subset, subset_split);

		double minHueVal, maxHueVal, minSatVal, maxSatVal, minValVal, maxValVal;
		minMaxLoc(subset_split[0], &minHueVal, &maxHueVal, NULL, NULL);
		minMaxLoc(subset_split[1], &minSatVal, &maxSatVal, NULL, NULL);
		minMaxLoc(subset_split[2], &minValVal, &maxValVal, NULL, NULL);

		printf("H: %f %f, S: %f %f, V: %f %f\n", minHueVal, maxHueVal,
			minSatVal, maxSatVal, minValVal, maxValVal);
	}
	fflush(stdout);
}

void NewMinH(int a, void *ptr) { minHSVs[target_mode][0] = MinH; }
void NewMinS(int a, void *ptr) { minHSVs[target_mode][1] = MinS; }
void NewMinV(int a, void *ptr) { minHSVs[target_mode][2] = MinV; }

void NewMaxH(int a, void *ptr) { maxHSVs[target_mode][0] = MaxH; }
void NewMaxS(int a, void *ptr) { maxHSVs[target_mode][1] = MaxS; }
void NewMaxV(int a, void *ptr) { maxHSVs[target_mode][2] = MaxV; }

void NewRatioMin(int a, void *ptr) { RatioMins[target_mode] = RatioMin; }
void NewRatioMax(int a, void *ptr) { RatioMaxs[target_mode] = RatioMax; }
void NewMinContourArea(int a, void *ptr) { ContourAreaMins[target_mode] = ContourAreaMin; }
void NewMaxContourArea(int a, void *ptr) { ContourAreaMaxs[target_mode] = ContourAreaMax; }
void NewCropTop(int a, void *ptr) { CropTops[target_mode] = crop_top; }
void NewCropBottom(int a, void *ptr) { CropBottoms[target_mode] = crop_bottom; }

void createWidgets()
{
	createTrackbar("MinH", "MainWindow", &MinH, 179, NewMinH);//Hue(0-179)
	createTrackbar("MaxH", "MainWindow", &MaxH, 179, NewMaxH);

	createTrackbar("MinS", "MainWindow", &MinS, 255, NewMinS);//Saturation(0-255)
	createTrackbar("MaxS", "MainWindow", &MaxS, 255, NewMaxS);

	createTrackbar("MinV", "MainWindow", &MinV, 255, NewMinV);//Value(0-255)
	createTrackbar("MaxV", "MainWindow", &MaxV, 255, NewMaxV);

	createTrackbar("Ratio Min", "MainWindow", &RatioMin, 200, NewRatioMin);
	createTrackbar("Ratio Max", "MainWindow", &RatioMax, 700, NewRatioMax);

	createTrackbar("Area Min", "MainWindow", &ContourAreaMin, 2000, NewMinContourArea);
	createTrackbar("Area Max", "MainWindow", &ContourAreaMax, 2000, NewMaxContourArea);

	createTrackbar("Crop Top", "MainWindow", &crop_top, screen_height - 1, NewCropTop);
	createTrackbar("Crop Bottom", "MainWindow", &crop_bottom, screen_height - 1, NewCropBottom);

	setMouseCallback("HSVImage", on_mouse, 0);
}

void processPressedKey(char key)
{
	static bool firstTime = true;

	if (firstTime) {
		printf("Key usage:\n'h': Switch to tracking High fuel target\n");
		printf("'l': Switch to tracking Low fuel target\n");
		printf("'g': Switch to tracking fuel on ground\n");
		printf("'b': Switch to tracking hopper target\n");
		printf("'G': Switch to tracking gear pin\n");
		printf("'f': Switch to tracking feeder target\n");
		printf("'z': Switch to tracking gear on ground target\n");
		printf("'A': Send UDP image always\n");
		firstTime = false;
	}

	if (key == 'h') {
		target_mode = HIGH_FUEL_TARGET;
		printf("Tracking High Boiler\n");
	}
	else if (key == 'l'){
		target_mode = LOW_FUEL_TARGET;
		printf("Tracking Low Boiler\n");
	}
	else if (key == 'g') {
		target_mode = FUEL_ON_GROUND_TARGET;
		printf("Tracking Fuel On Ground\n");
	}
	else if (key == 'b'){
		target_mode = HOPPER_TARGET;
		printf("Tracking Hopper\n");
	}
	else if (key == 'r'){
		target_mode = ROPE_TARGET;
		printf("Tracking Rope\n");
	}
	else if (key == 'G'){
		target_mode = GEAR_PIN_TARGET;
		printf("Tracking Gear Pin\n");
	}
	else if (key == 'f'){
		target_mode = FEEDER_TARGET;
		printf("Tracking Feeder\n");
	}
	else if (key == 'z'){
		target_mode = GEAR_ON_GROUND_TARGET;
		printf("Tracking Feeder\n");
	}
	else if (key == 'A'){
		image_request_freq = 10; // always send image
		printf("Image Freq is 255\n");
	}
	else if (key == 'c'){
		image_request_crop = !image_request_crop; // always send image
		printf("Toggled Image Crop to %d\n", image_request_crop);
	}
	else {
		printf("invalid key=%c %d \n", key, key);
	}

	loadScrollBars();
}

//int _tmain(int argc, _TCHAR* argv[])
int main(int argc, char* argv[])
{
	double oldTimeNow = 0;
	//char buf[1500];
	Mat originalRawImage;

	if (argc == 2) {
		char fname[64];
		memcpy(fname, argv[1], strlen( argv[1] ) );
		fname[strlen(argv[1])] = '\0';
		croppedRawImage = imread(fname);
		croppedRawImage.copyTo(originalRawImage);
	}
	else {
		printf("usage: SteamWorks debugmode filename\n");
		exit(0);
	}

	// the following are the latest empirical values
	minHSVs[HIGH_FUEL_TARGET] = Scalar(0, 14, 0);
	minHSVs[FUEL_ON_GROUND_TARGET] = Scalar(67, 12, 205);
	minHSVs[HOPPER_TARGET] = Scalar(67, 12, 205);
	minHSVs[ROPE_TARGET] = Scalar(16, 24, 144);
	minHSVs[GEAR_PIN_TARGET] = Scalar(16, 136, 98);
	minHSVs[GEAR_ON_GROUND_TARGET] = Scalar(13, 170, 91);

	if (alliance_color == RED) {
		minHSVs[LOW_FUEL_TARGET] = Scalar(67, 12, 205);
		maxHSVs[LOW_FUEL_TARGET] = Scalar(67, 12, 205);
		minHSVs[FEEDER_TARGET] = Scalar(67, 12, 205);
		maxHSVs[FEEDER_TARGET] = Scalar(67, 12, 205);
	}
	else {
		minHSVs[LOW_FUEL_TARGET] = Scalar(67, 12, 205);
		maxHSVs[LOW_FUEL_TARGET] = Scalar(67, 12, 205);
		minHSVs[FEEDER_TARGET] = Scalar(67, 12, 205);
		maxHSVs[FEEDER_TARGET] = Scalar(67, 12, 205);
	}
	maxHSVs[HIGH_FUEL_TARGET] = Scalar(115, 102, 255);
	maxHSVs[HOPPER_TARGET] = Scalar(115, 102, 255);
	maxHSVs[FUEL_ON_GROUND_TARGET] = Scalar(67, 12, 205);
	maxHSVs[HOPPER_TARGET] = Scalar(67, 12, 205);
	maxHSVs[ROPE_TARGET] = Scalar(25, 47, 189);
	maxHSVs[GEAR_PIN_TARGET] = Scalar(24, 189, 176); // found
	maxHSVs[GEAR_ON_GROUND_TARGET] = Scalar(30, 255, 255); // found

	double oldLogTime = 0.0; // sec
	if (1) {
		namedWindow("MainWindow", CV_WINDOW_NORMAL);
		namedWindow("RawImage", CV_WINDOW_AUTOSIZE);
		namedWindow("BinaryImage", CV_WINDOW_AUTOSIZE);
		namedWindow("HSVImage", CV_WINDOW_AUTOSIZE);

		cvMoveWindow("MainWindow", 0, 0);
		cvMoveWindow("RawImage", 0, 0);
		cvMoveWindow("BinaryImage", 640, 320);
		cvMoveWindow("HSVImage", 640, 0);

		createWidgets();
		loadScrollBars();
	}
	int desiredWidth = 1280, desiredheight = 10;
	cvResizeWindow("MainWindow", desiredWidth, desiredheight);

	while (1) { // forever
		originalRawImage.copyTo(croppedRawImage);
		originalRawImage.copyTo(rawImage);

		Track();

		//struct timeval tv;
		//gettimeofday(&tv, NULL);
		//double timeNow = (double)((double)tv.tv_sec +
			//((double)tv.tv_usec) / 1000000.0);

		imshow("RawImage", croppedRawImage);
		imshow("BinaryImage", binaryImage);
		imshow("HSVImage", HSVImage);

		//dt = timeNow - oldTimeNow;
		//printf("%f fps\n", 1.0/dt );	
		//oldTimeNow = timeNow;

		char key = (char)waitKey(10);
		//printf("key=%c\n", key );

		if(key != 255) {
			processPressedKey(key);
		}
	} // while
}

