#include<iostream>
#include <pthread.h>
#include<stdio.h>
#include<stdlib.h>
#include<vector>
#include<sys/time.h>
#include<time.h>
#include<opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>

#include "Arduino.h"
#include "UdpSocketLinux.h"
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

class UdpSocket Net;

const int ELEV_PIN = GPIO3;
const int AZ_PIN = GPIO5;
const int RING_LIGHT_PIN = GPIO7;
const int GO_LEFT_PIN = GPIO6;
const int GO_RIGHT_PIN = GPIO9;
const int CENTERED_PIN = GPIO10;

int debug_mode=1;

int image_request_type=HSV_TYPE;
int image_request_crop=CROPPED;
int image_request_freq=1; // period between transmits, 0 means never send, 255 means always

int currentState=LEFT_DEADBAND_STATE;
int hopper_side=1; // 0-left, 1-right
int send_error_mode=1;
int target_mode=HIGH_FUEL_TARGET; // GEAR_PIN_TARGET; // HIGH_FUEL_TARGET;
int track_error_x=0; // pixels
int range_to_target=0; // feet
int cg_x = 0;
int cg_y = 0;

int crop_top=10;
int crop_bottom=screen_height-10;

int MinH = 0;
int MaxH = 179;//180?
int MinS = 0;
int MaxS = 255;//256
int MinV = 0;
int MaxV = 255;
int ContourAreaMin = 10;
int ContourAreaMax = 255;
int Threshold=127;
	
// upper goal variables
Scalar minHSVs[TOTAL_TARGETS];
Scalar maxHSVs[TOTAL_TARGETS];
int ContourAreaMins[TOTAL_TARGETS] = { 90, 0, 0, 0, 0, 1100, 0};
int ContourAreaMaxs[TOTAL_TARGETS] = {255, 255, 255, 255, 255, 2200, 255 };
int Thresholds[TOTAL_TARGETS]={ 127, 127, 127, 127, 127, 127, 127};
int GimbalAngles[TOTAL_TARGETS]= { 9, 9, 9, 9, 9, 9, 9 };
int CropTops[TOTAL_TARGETS]= { 200, 9, 9, 9, 9, 252, 9 };
int CropBottoms[TOTAL_TARGETS]= { 400, 400, 400, 400, 400, 395, 400 };

int confidence_level; // 0-100
int numContoursFound=0;
bool imagesLocked=true;
#ifdef USE_PWM
//int gimbal_mode = 0;
int gimbal_el_angle = 0; // truncated deg
int gimbal_az_angle = 0; // truncated deg
int elevation_duty=ELEV_SERVO_ZERO_DEG;
int azimuth_duty=AZ_SERVO_ZERO_DEG;
#endif

int alliance_color = 0;
int match_time=0; // int seconds
double dt=0.0;

Mat rawImage;
Mat HSVImage;
Mat HSVImageFiltered;
Mat binaryImage;
Mat croppedRawImage;

pthread_t image_thread;
pthread_t udp_thread;
void SendImage();

void *image_fun( void *arg )
{
	struct timeval tv;
	double old_send_image_time=0.0;

	while(1) {
		while( imagesLocked ) {
			usleep( 100000 );
		}
		gettimeofday( &tv, NULL );
		double timeNow = (double)((double)tv.tv_sec + 
			((double)tv.tv_usec)/1000000.0);
			
		if( image_request_freq == 255 ) // send as fast as possible
			SendImage(); 
		else if( image_request_freq > 0 ) { // send as fast as possible
			if( ( timeNow - old_send_image_time ) > image_request_freq ) {
				SendImage(); 
				old_send_image_time = timeNow;
			}
		}
		
	}
}

void *udp_fun( void *arg )
{
	char buf[1500];
	while( 1 ) {
		int len_msg = Net.getDatagram( buf, 1500 );
//printf("line=%d\n", __LINE__ );
			
		// decode the string into a two byte message
		unsigned char header;
		if( len_msg > 0 ) {
			memcpy( &header, buf, 1 );
printf("header: 0x%x\n", header );
			if( header == ERROR_REQUEST_HEADER ) // command to send goal commands
			{
				memcpy( &send_error_mode, buf+1, 1 );
			}
			else if( header == IMAGE_REQUEST_HEADER ) // command to not track goal
			{
				memcpy( &image_request_type, buf+1, 1 );
				memcpy( &image_request_crop, buf+2, 1 );
				memcpy( &image_request_freq, buf+3, 1 );
			}
#ifdef USE_PWM
			else if( header == GIMBAL_REQUEST_HEADER )
			{
				memcpy( &gimbal_el_angle, buf+1, 1 ); // angle
				memcpy( &gimbal_az_angle, buf+2, 2 ); // angle in truncated degrees
				
				// convert degrees to servo commands. Interpolate
				elevation_duty = EL_SERVO_MIN + 
					(EL_SERVO_MAX-EL_SERVO_MIN)*gimbal_el_angle/90.0;
				digitalWrite( ELEV_PIN, elevation_duty );
				azimuth_duty = AZ_SERVO_MIN + 
					(AZ_SERVO_MAX-AZ_SERVO_MIN)*gimbal_az_angle/180.0;
				digitalWrite( AZ_PIN, azimuth_duty );
			}
#endif
			else if( header == TARGET_MODE_HEADER ) {
				memcpy( &target_mode, buf+1, 1 );
				if( target_mode == HOPPER_TARGET ) 
					memcpy( &hopper_side, buf+1, 1 );

				if( ( target_mode == HIGH_FUEL_TARGET ) || 
				    ( target_mode == GEAR_PIN_TARGET ) )
					digitalWrite( RING_LIGHT_PIN, HIGH );
				else
					digitalWrite( RING_LIGHT_PIN, LOW );
			}
			else if( header == INITIALIZATION_HEADER) {
				memcpy( &alliance_color, buf+1, 1 );
				if( alliance_color == RED ) {
					minHSVs[LOW_FUEL_TARGET] = Scalar(67,12,205);
					maxHSVs[LOW_FUEL_TARGET] = Scalar(67,12,205);
					minHSVs[FEEDER_TARGET] = Scalar(67,12,205);
					maxHSVs[FEEDER_TARGET] = Scalar(67,12,205);
				}
				else {
					minHSVs[LOW_FUEL_TARGET] = Scalar(67,12,205);
					maxHSVs[LOW_FUEL_TARGET] = Scalar(67,12,205);
					minHSVs[FEEDER_TARGET] = Scalar(67,12,205);
					maxHSVs[FEEDER_TARGET] = Scalar(67,12,205);
				}
				memcpy( &match_time, buf+2, 1 );
			}
		} // if len > 0
		usleep( 10000 );
	} // while
}

void initialize_Acadia()
{
	pinMode( RING_LIGHT_PIN, OUTPUT );
	pinMode( GO_LEFT_PIN, OUTPUT ); // red
	pinMode( GO_RIGHT_PIN, OUTPUT ); // green
	pinMode( CENTERED_PIN, OUTPUT );
	
#ifdef USE_PWM
	pwmfreq_set( ELEV_PIN, 50 );
	analogWrite( ELEV_PIN, elevation_duty );	

	pwmfreq_set( AZ_PIN, 50 );
	analogWrite( AZ_PIN, azimuth_duty );	
#endif	
	if( ( target_mode == HIGH_FUEL_TARGET ) || ( target_mode == GEAR_PIN_TARGET ) )
		digitalWrite( RING_LIGHT_PIN, HIGH );
	else
		digitalWrite( RING_LIGHT_PIN, LOW );
	digitalWrite( GO_LEFT_PIN, HIGH );
	digitalWrite( GO_RIGHT_PIN, LOW );
	digitalWrite( CENTERED_PIN, HIGH );
}

void SendImage()
{
	const int MAX_IMAGE_SIZE=160000;
	unsigned char buffer[MAX_IMAGE_SIZE];
	int params[4];
	memcpy( buffer, &IMAGE_HEADER, 1 );
	unsigned char fps=(unsigned char)(1.0/dt);
	memcpy( buffer+1, &fps, 1 );
	memcpy( buffer+2, &alliance_color, 1 );
	memcpy( buffer+3, &cg_x, 2 );
	memcpy( buffer+5, &cg_y, 2 );
	int len=0;

//printf("image_request_type=%d\n", image_request_type );
	vector<unsigned char> compressedImage;
	if( imagesLocked ) return;
	
	if( image_request_type == HSV_TYPE ) {
		imencode(".jpg", HSVImageFiltered, compressedImage );
	}
	else if( image_request_type == BINARY_TYPE ) {
		imencode(".jpg", binaryImage, compressedImage );
	}	
	else if( image_request_type == RAW_TYPE ) {
		imencode(".jpg", croppedRawImage, compressedImage );
	}
	
	len = compressedImage.size(); // bytes
	if( len >= MAX_IMAGE_SIZE ) return;

//printf("len=%d\n", len );
	memcpy( buffer+7, &len, 4 );
	//buffer[3] = reinterpret_cast<uchar*> (&compressedImage[0]); // set pointers for better speed
	memcpy( buffer+11, compressedImage.data(), len ); // this is expensive, just set pointers
	Net.putDatagram( (char *)buffer, len, ACADIA_PORT );
}

void SendErrorMessage( int er )
{
	struct ErrorMessage err;
	err.header = ERROR_HEADER;
	err.range = range_to_target;
	err.horizontal_error = er;
	err.target_mode = target_mode;
	err.confidence = (int)(confidence_level*100.0);
	
	const int error_len = sizeof( struct ErrorMessage );
	
	unsigned char buffer[1500];
	memcpy( buffer, &err, error_len );
    	Net.putDatagram( (char *)buffer, error_len, ACADIA_PORT );
}

void ControlDirectionLEDs()
{
	// illuminate the LEDs as a function of the track error
//printf("CurrentState=%d\n", currentState );
	if( currentState == RIGHT_DEADBAND_STATE ) {
		digitalWrite( GO_LEFT_PIN, HIGH );
		digitalWrite( GO_RIGHT_PIN, LOW );
		digitalWrite( CENTERED_PIN, HIGH );
	}
	else if( currentState == LEFT_DEADBAND_STATE ) {
		digitalWrite( GO_LEFT_PIN, LOW );
		digitalWrite( GO_RIGHT_PIN, HIGH );
		digitalWrite( CENTERED_PIN, HIGH );
	}
	else if( currentState == CENTERED_DEADBAND_STATE ) {
		digitalWrite( GO_LEFT_PIN, HIGH );
		digitalWrite( GO_RIGHT_PIN, HIGH );
		digitalWrite( CENTERED_PIN, LOW );
	}
}

int handle_HIGH_FUEL_TARGET_TrackError(Rect boundRect)
{
	track_error_x = boundRect.x + (boundRect.width/2) - screen_widthO2;
//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height/boundRect.width;
printf("ratio = %f\n", ratio  );
	if( ( ratio > 0.3 ) && ( ratio < 0.4 ) ) {
		numContoursFound += 1;
		confidence_level += 0.5; //
	}
	else if( ( ratio > 0.1 ) && ( ratio < 0.2 ) ) {
		numContoursFound += 1;
		confidence_level += 0.5; //
	}

	// use a 5th order poly to estimate range ????
	double Gain=1.0;
	range_to_target = 144.0 + Gain*boundRect.width; // inches
}

int handle_LOW_FUEL_TARGET_TrackError(Rect boundRect)
{
	static double sum_x=0;
	if( numContoursFound == 0 ) sum_x = 0;
//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height/boundRect.width;
printf("ratio = %f\n", ratio  );
	// the lower goal contours are small and 8.75 tall x 3" wide = rati of 2.9
	if( ( ratio > 2.7 ) && ( ratio < 3.1 ) ) {
		numContoursFound += 1;
		confidence_level += 0.125; //
		// get the geometric center
		sum_x += boundRect.x;
		track_error_x = (double)(1.0*sum_x/numContoursFound) + boundRect.width;
	}

	// use a 5th order poly to estimate range ????
	double Gain=1.0;
	range_to_target = 44.0 + Gain*boundRect.width; // inches
}

int handle_FUEL_ON_GROUND_TARGET_TrackError(Rect boundRect)
{
	static int sum_x = 0;
	if( numContoursFound == 0 ) sum_x = 0;
//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height/boundRect.width;
printf("ratio = %f\n", ratio  );
	if( ( ratio > 0.9 ) && ( ratio < 1.1 ) ) {
		numContoursFound += 1;
		confidence_level += 1.0; //
		sum_x += boundRect.x;
		track_error_x = (double)(1.0*sum_x/numContoursFound) + boundRect.width;
	}

	// use a 5th order poly to estimate range ????
	double Gain=1.0;
	range_to_target = 144.0 + Gain*boundRect.width; // inches
}

int handle_HOPPER_TARGET_TrackError(Rect boundRect)
{
	static int center_x=0;
	// this target should be tracking the left or right hopper. use hopper_side variable ????
//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height/boundRect.width;
printf("ratio = %f\n", ratio  );

	if( numContoursFound > 0 ) {
		if( boundRect.x < center_x ) {
			numContoursFound += 1;
			track_error_x = boundRect.x + (boundRect.width/2) - screen_widthO2;
		}
	}
	// hopper_side 0 means track the left hopper ????
	// hopper is 26.5x23.25 = rati oof 1.14
	else if( ( ratio > 1.0 ) && ( ratio < 1.3 ) ) {
		numContoursFound += 1;
		confidence_level += 1.0; //
		track_error_x = boundRect.x + (boundRect.width/2) - screen_widthO2;
	}

	// use a 5th order poly to estimate range ????
	double Gain=1.0;
	range_to_target = 144.0 + Gain*boundRect.width; // inches
}

int handle_ROPE_TARGET_TrackError(Rect boundRect)
{
//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height/boundRect.width;
printf("ratio = %f\n", ratio  );
	if( ( ratio > 0.3 ) && ( ratio < 0.4 ) ) {
		numContoursFound += 1;
		confidence_level = 1.0; //
		track_error_x = boundRect.x + (boundRect.width/2) - screen_widthO2;
	}

	// use a 5th order poly to estimate range ????
	double Gain=1.0;
	range_to_target = 144.0 + Gain*boundRect.width; // inches
}

int handle_GEAR_PIN_TARGET_TrackError(Rect boundRect)
{
	static int sum_x=0;
	
	if( numContoursFound == 0  ) sum_x = 0; 

//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the top or bottom tape
	double ratio = 1.0*boundRect.height/boundRect.width;
printf("ratio = %f\n", ratio  );
	if( ( ratio > 1.6 ) && ( ratio < 2.7 ) ) {
		numContoursFound += 1;
		confidence_level += 0.5; //
		sum_x += boundRect.x;
printf("numContoursFound = %d\n", numContoursFound );
		if( numContoursFound == 2 ) {
			track_error_x = sum_x/numContoursFound + boundRect.x;
printf("track_error_x = %d\n", track_error_x );
		}
	}

	// use a 5th order poly to estimate range ????
	double Gain=1.0;
	range_to_target = 144.0 + Gain*boundRect.width; // inches
}

int handle_FEEDER_TARGET_TrackError(Rect boundRect)
{
	// have the feeder guy hold up a large reflective triangle
	
//printf("Contour location cnt = %d: %d %d\n", cnt, boundRect.x, boundRect.y);
	// this could be the left or right feeder or both
	double ratio = 1.0*boundRect.height/boundRect.width;
printf("ratio = %f\n", ratio  );
	// feeder opening is 25" wide by 7.5" tall = 0.3
	if( ( ratio > 0.25 ) && ( ratio < 0.35 ) ) {
		numContoursFound += 1;
		confidence_level = 1.0; //
		track_error_x = boundRect.x + (boundRect.width/2) - screen_widthO2;
	}

	// use a 5th order poly to estimate range ????
	double Gain=1.0;
	range_to_target = 144.0 + Gain*boundRect.width; // inches
}

void Track()
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

//printf("targetMode=%d: MaxHSV %f %f %f\n", target_mode, 
	//maxHSVs[target_mode][0], maxHSVs[target_mode][1], maxHSVs[target_mode][2]);
//printf("crop=%d %d\n", CropTops[target_mode], CropBottoms[target_mode] );
	const Rect roi( 0, CropTops[target_mode], screen_width-1, 
		CropBottoms[target_mode]-CropTops[target_mode]-1);
	Mat image_roi = rawImage(roi);

	imagesLocked = true;

	image_roi.copyTo( croppedRawImage);
	cvtColor( croppedRawImage, HSVImage, CV_BGR2HSV);
	
	// filter Hue values outside of this range
	inRange(HSVImage, minHSVs[target_mode], maxHSVs[target_mode], HSVImageFiltered);
                
    	// convert the filtered image to a binary format for contouring
    	threshold( HSVImageFiltered, binaryImage, Thresholds[target_mode], 255, 0);

	imagesLocked = false;
	
	// save the binaryImage because the findContours() will destroy it
	Mat tempBinaryImage = binaryImage.clone();
	findContours( tempBinaryImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) ); 

	confidence_level = 0.0;
	numContoursFound = 0;

//printf("Contour size=%d\n", contours.size() );
	if (contours.size()>0) {
		vector<Point> contours_poly ( contours.size() );

		// Approximate contours to polygons + get bounding rects and circles
		for ( int cnt=0; cnt<contours.size(); ++cnt ) {
				approxPolyDP( Mat(contours[cnt] ), contours_poly, 3, true);
				int contour_area = contourArea(contours[cnt] );
//printf( "contour_area = %d\n", contour_area );
				
				if( ( contour_area > ContourAreaMins[target_mode] ) && 
 				    (contour_area < ContourAreaMaxs[target_mode] ) ) { // estimate
					Rect boundRect = boundingRect ( Mat( contours_poly) );
if( debug_mode == 1 ) {
printf( "in contour_area = %d\n", contour_area );
printf( "in boundRect = x=%d, width=%d\n", boundRect.x, boundRect.width );
}
					if ( debug_mode == 1)
						rectangle (HSVImageFiltered, boundRect, (0,255,0), 2);
				
					if( target_mode == HIGH_FUEL_TARGET ) 
						handle_HIGH_FUEL_TARGET_TrackError(boundRect);
					else if( target_mode == LOW_FUEL_TARGET ) 
						handle_LOW_FUEL_TARGET_TrackError(boundRect);
					else if( target_mode == FUEL_ON_GROUND_TARGET ) 
						handle_FUEL_ON_GROUND_TARGET_TrackError(boundRect);
					else if( target_mode == HOPPER_TARGET ) 
						handle_HOPPER_TARGET_TrackError(boundRect);
					else if( target_mode == ROPE_TARGET ) 
						handle_ROPE_TARGET_TrackError(boundRect);
					else if( target_mode == GEAR_PIN_TARGET ) 
						handle_GEAR_PIN_TARGET_TrackError(boundRect);
					else if( target_mode == FEEDER_TARGET ) 
						handle_FEEDER_TARGET_TrackError(boundRect);

printf("error=%d\n", track_error_x );
					if ( abs(track_error_x) < DeadBand) {
						//printf( "Center %d\n", track_error_x);
						currentState = CENTERED_DEADBAND_STATE;
					}
					else if ( track_error_x< -DeadBand) {
						//printf( "LEFT %d\n", track_error_x);
						currentState = LEFT_DEADBAND_STATE;
					}
					else {
						//printf( "RIGHT %d\n", track_error_x);
						currentState = RIGHT_DEADBAND_STATE;
					}//if
					ControlDirectionLEDs();
					
					if( debug_mode == 1 ) {
						drawContours( croppedRawImage, contours_poly, 
							cnt, Scalar (0,255,0), 2 ); // last is thickness
					}
				} // if contour_area
		}//for cnt
	}
}

void on_mouse(int event, int x, int y, int flags, void* userdata)
{
	static int start_x=0, start_y=0;
	static int end_x=0, end_y=0;
	bool endComplete = false;
	//const int channels[] = { 0, 1, 2 };
	
    	if( event == EVENT_LBUTTONDOWN ) {
        	printf("Start Mouse Position: %d %d\n", x, y );
        	start_x = x;
		start_y = y;
	}
    	else if( event == EVENT_LBUTTONUP ) {
        	printf("End Mouse Position: %d, %d\n", x, y );
        	end_x = x;
		end_y = y;
		endComplete = true;
	}
	// now go into the imag array and find the histogram of the area
	if( endComplete ) {
		endComplete = false;
		
		// get the subset image from the numpy array
		const Rect roi( start_x, start_y, end_x-start_x, end_y-start_y );
printf("%d %d %d %d\n", start_x, start_y, end_x, end_y );
printf("area=%d\n", (end_x-start_x)*( end_y-start_y) );
	
		// this feature is only needed for HSV values
		Mat subset;
		Mat image_roi = HSVImage(roi);
		image_roi.copyTo( subset );
		
		vector<Mat> subset_split(3);
		split( subset, subset_split );
//printf("Line=%d\n", __LINE__ );
		
		double minHueVal,maxHueVal,minSatVal,maxSatVal,minValVal,maxValVal;
		minMaxLoc( subset_split[0], &minHueVal, &maxHueVal, NULL, NULL );
		minMaxLoc( subset_split[1], &minSatVal, &maxSatVal, NULL, NULL );
		minMaxLoc( subset_split[2], &minValVal, &maxValVal, NULL, NULL );

printf("H: %f %f, S: %f %f, V: %f %f\n", minHueVal, maxHueVal,
	minSatVal, maxSatVal, minValVal, maxValVal );
	}
}

// ???? call this from recevied UDP packet too
// ???? can I write all images to the same window and have them stack???
void initializeNewTargetMode()
{
	setTrackbarPos( "MinH", "MainWindow", minHSVs[target_mode][0] );
	setTrackbarPos( "MinS", "MainWindow", minHSVs[target_mode][1] );
	setTrackbarPos( "MinV", "MainWindow", minHSVs[target_mode][2] );
	setTrackbarPos( "MaxH", "MainWindow", maxHSVs[target_mode][0] );
	setTrackbarPos( "MaxS", "MainWindow", maxHSVs[target_mode][1] );
	setTrackbarPos( "MaxV", "MainWindow", maxHSVs[target_mode][2] );
	
	setTrackbarPos( "Area Min", "MainWindow", ContourAreaMins[target_mode] );
	setTrackbarPos( "Area Max", "MainWindow", ContourAreaMaxs[target_mode] );

	setTrackbarPos( "Threashold", "MainWindow", Thresholds[target_mode] );
	
	initializeNewTargetMode();
}

void NewMinH( int a, void *ptr ) { minHSVs[target_mode][0] = MinH; }
void NewMinS( int a, void *ptr ) { minHSVs[target_mode][1] = MinS; }
void NewMinV( int a, void *ptr ) { minHSVs[target_mode][2] = MinV; }

void NewMaxH( int a, void *ptr ) { maxHSVs[target_mode][0] = MaxH; }
void NewMaxS( int a, void *ptr ) { maxHSVs[target_mode][1] = MaxS; }
void NewMaxV( int a, void *ptr ) { maxHSVs[target_mode][2] = MaxV; }

void NewThreshold( int a, void *ptr ) { Thresholds[target_mode] = Threshold; }
void NewMinContourArea( int a, void *ptr ) { ContourAreaMins[target_mode] = ContourAreaMin; }
void NewMaxContourArea( int a, void *ptr ) { ContourAreaMaxs[target_mode] = ContourAreaMax; }
#ifdef USE_PWM
void NewGimbalElAngle( int a, void *ptr ) { GimbalElAngles[target_mode] = gimbal_el_angle; }
void NewGimbalAzAngle( int a, void *ptr ) { GimbalAzAngles[target_mode] = gimbal_az_angle; }
#endif
void NewCropTop( int a, void *ptr ) { CropTops[target_mode] = crop_top; }
void NewCropBottom( int a, void *ptr ) { CropBottoms[target_mode] = crop_bottom; }

void createWidgets()
{
	createTrackbar("MinH","MainWindow",&MinH,179, NewMinH);//Hue(0-179)
	createTrackbar("MaxH","MainWindow",&MaxH,179, NewMaxH);

	createTrackbar("MinS","MainWindow",&MinS,255, NewMaxS);//Saturation(0-255)
	createTrackbar("MaxS","MainWindow",&MaxS,255, NewMaxS);

	createTrackbar("MinV","MainWindow",&MinV,255, NewMaxV);//Value(0-255)
	createTrackbar("MaxV","MainWindow",&MaxV,255, NewMaxV);

	createTrackbar("Threshold","MainWindow",&Threshold,255, NewThreshold);

	createTrackbar("Area Min","MainWindow",&ContourAreaMin,20000, NewMinContourArea);
	createTrackbar("Area Max","MainWindow",&ContourAreaMax,20000, NewMaxContourArea);

	createTrackbar("Crop Top","MainWindow", &crop_top, screen_height-1, NewCropTop );
	createTrackbar("Crop Bottom","MainWindow", &crop_bottom, screen_height-1, NewCropBottom );
	
#ifdef USE_PWM
	createTrackbar("Gimbal Elev","MainWindow",&gimbal_el_angle,90, NewGimbalElAngle);
	createTrackbar("Gimbal Az","MainWindow",&gimbal_az_angle,90, NewGimbalAzAngle);
#endif
		
	setMouseCallback("HSVImage", on_mouse, 0);
}

void processPressedKey( char key )
{
	static bool firstTime=true;
	
	if( firstTime ) {
		printf("Key usage:\n'h': Switch to tracking High fuel target\n" );
		printf("'l': Switch to tracking Low fuel target\n" );
		printf("'f': Switch to tracking fuel on ground\n" );
		printf("'h': Switch to tracking hopper target\n" );
		printf("'G': Switch to tracking gear pin\n" );
		printf("'f': Switch to tracking feeder target\n" );
		printf("'A': Send UDP image always\n" );
		firstTime = false;
	}
	
	if( key == 'h' ) 
		target_mode = HIGH_FUEL_TARGET;
	else if( key == 'l' )
		target_mode = LOW_FUEL_TARGET;
	else if( key == 'g' ) 
		target_mode = FUEL_ON_GROUND_TARGET;	
	else if( key == 'h' )
		target_mode = HOPPER_TARGET;
	else if( key == 'r' )
		target_mode = ROPE_TARGET;
	else if( key == 'G' )
		target_mode = GEAR_PIN_TARGET;
	else if( key == 'f' )
		target_mode = FEEDER_TARGET;
	else if( key == 'A' )
		image_request_freq = 255; // always send image
	else {
		printf( "invalid key=%c %d \n", key, key );
	}

	if( ( target_mode == HIGH_FUEL_TARGET ) || ( target_mode == GEAR_PIN_TARGET ) )
		digitalWrite( RING_LIGHT_PIN, HIGH );
	else
		digitalWrite( RING_LIGHT_PIN, LOW );
}

int main( int argc, char *argv[] )
{
	double oldTimeNow=0;
	//char buf[1500];
	Mat originalRawImage;
	bool useFileImage=false;
	
	if( argc > 1 ) {
		croppedRawImage = imread( argv[1] );
		originalRawImage = croppedRawImage;
		useFileImage = 1;
		debug_mode = 1;
	}
	
	Net.init( ACADIA_PORT, ACADIA_IP );
	
	int err = pthread_create( &udp_thread, NULL, udp_fun, NULL );
	if( err != 0 ) {
		printf("Error: udp_thread is not running\n" );
		exit(0);
	}
		
	err = pthread_create( &image_thread, NULL, image_fun, NULL );
	if( err != 0 ) {
		printf("Error: image_thread is not running\n" );
		exit(0);
	}

	// the following are the latest empirical values
	minHSVs[HIGH_FUEL_TARGET] = Scalar(0,14,0);
	minHSVs[FUEL_ON_GROUND_TARGET] = Scalar(67,12,205);
	minHSVs[HOPPER_TARGET] = Scalar(67,12,205);
	minHSVs[ROPE_TARGET] = Scalar(67,12,205);
	minHSVs[GEAR_PIN_TARGET] = Scalar(0,0,164);

	if( alliance_color == RED ) {
		minHSVs[LOW_FUEL_TARGET] = Scalar(67,12,205);
		maxHSVs[LOW_FUEL_TARGET] = Scalar(67,12,205);
		minHSVs[FEEDER_TARGET] = Scalar(67,12,205);
		maxHSVs[FEEDER_TARGET] = Scalar(67,12,205);
	}
	else {
		minHSVs[LOW_FUEL_TARGET] = Scalar(67,12,205);
		maxHSVs[LOW_FUEL_TARGET] = Scalar(67,12,205);
		minHSVs[FEEDER_TARGET] = Scalar(67,12,205);
		maxHSVs[FEEDER_TARGET] = Scalar(67,12,205);
	}
	maxHSVs[HIGH_FUEL_TARGET] = Scalar(115,102,255);
	maxHSVs[HOPPER_TARGET] = Scalar(115,102,255);
	maxHSVs[FUEL_ON_GROUND_TARGET] = Scalar(67,12,205);
	maxHSVs[HOPPER_TARGET] = Scalar(67,12,205);
	maxHSVs[ROPE_TARGET] = Scalar(90,12,205);
	maxHSVs[GEAR_PIN_TARGET] = Scalar(91,210,255); // found

	crop_top = CropTops[target_mode];
	crop_bottom = CropBottoms[target_mode];
	MinH = minHSVs[target_mode][0];
	MinS = minHSVs[target_mode][1];
	MinV = minHSVs[target_mode][2];
	MaxH = maxHSVs[target_mode][0];
	MaxS = maxHSVs[target_mode][1];
	MaxV = maxHSVs[target_mode][2];
	ContourAreaMin = ContourAreaMins[target_mode];
	ContourAreaMax = ContourAreaMaxs[target_mode];
	Threshold = Thresholds[target_mode];

	init(); // Acadia initialization
	initialize_Acadia();
	
	VideoCapture *capture;
	for( int i=0; i < 10; i ++ ) {
		capture = new VideoCapture(i);
    		if(!capture->isOpened()){
        		cout << "Failed to connect to the camera %d" << i << endl;
    		}
		else
		{
			cout<< "Obtained camera frame! i=" << i << endl;
			break;
		}
	}
    	int ret_val; 
    	ret_val = capture->set( CV_CAP_PROP_FRAME_WIDTH,  screen_width );
    	ret_val = capture->set( CV_CAP_PROP_FRAME_HEIGHT, screen_height );

//printf("line=%d\n", __LINE__ );
	
	double oldLogTime=0.0; // sec
	if( debug_mode == 1 ) {
		namedWindow( "MainWindow", CV_WINDOW_AUTOSIZE );
		namedWindow( "RawImage", CV_WINDOW_AUTOSIZE );
		namedWindow( "BinaryImage", CV_WINDOW_AUTOSIZE );
		namedWindow( "HSVImage", CV_WINDOW_AUTOSIZE );
	
		cvMoveWindow( "MainWindow", 0, 0 );
		cvMoveWindow( "RawImage", 0, 0 );
		cvMoveWindow( "BinaryImage", 640, 320 );
		cvMoveWindow( "HSVImage", 640, 0 );
	
		createWidgets();
	}

	while(1) { // forever
//printf("line=%d\n", __LINE__ );
		if( useFileImage == 0 )
			*capture >> rawImage;
		else {
			croppedRawImage = originalRawImage;
			rawImage = originalRawImage;
		}
        
		Track();
//printf("line=%d\n", __LINE__ );

		struct timeval tv;
		gettimeofday( &tv, NULL );
		double timeNow = (double)((double)tv.tv_sec + 
			((double)tv.tv_usec)/1000000.0);

		
//printf("line=%d\n", __LINE__ );
		// save the raw cropped image no matter which mode the user wants
		if( ( (timeNow - oldLogTime) > logTimeInterval ) ) { // save this image to a file for later processing
			char temp[128];
			time_t rawtime;
			struct tm *timeinfo;
			char buffer[80];
			const char *fnames[] = { "HighGoal", "LowGoal", 
				"FuelOnGround", "Hopper", "Rope", "Gear",
				"Feeder" };

			time (&rawtime );
			timeinfo = localtime( &rawtime );
			strftime( buffer, 80, "%a_%I_%M_%S_%p", timeinfo );
			sprintf( temp, (char *)"/home/linaro/Robotics2017/Logs/%s_err%d_time%s_%s.png", 
				fnames[target_mode], track_error_x - screen_widthO2,
				buffer, colors[alliance_color] );

			imwrite( (char *)temp, croppedRawImage );
			oldLogTime = timeNow; 
		}
		
//printf("line=%d\n", __LINE__ );
		if( debug_mode == 1) { // Show in a window
            		imshow("RawImage", croppedRawImage);
            		imshow("BinaryImage", binaryImage);
            		imshow("HSVImage", HSVImage );
		}
		dt = timeNow - oldTimeNow;
//printf("%f fps\n", 1.0/dt );	
		oldTimeNow = timeNow;
			
		char key = (char)waitKey(10);
//printf("key=%c\n", key );

		if( ( key != 255 ) && ( debug_mode == 1 ) ) {
			processPressedKey( key );
		}
	} // while
}
