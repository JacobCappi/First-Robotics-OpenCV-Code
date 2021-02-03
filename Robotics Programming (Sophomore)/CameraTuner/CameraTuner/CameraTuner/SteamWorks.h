const int DeadBand = 10;
const int DeadBandO2 = DeadBand/2;

const int LEFT_DEADBAND_STATE=1;
const int RIGHT_DEADBAND_STATE=2;
const int CENTERED_DEADBAND_STATE=3;

const int ACADIA_PORT = 5800; // use 5800-5810
const int IMAGE_PORT = 5801; // use 5800-5810

// Brewer may use two cameras
const int ONE_CAMERA = 1; 
const int FRONT_CAMERA = 2; 
const int BACK_CAMERA = 3; 
/*
#define TEAM_NUMBER 0

#if ( TEAM_NUMBER == 3959  )
const char ACADIA_IP[] = "10.39.59.255";
#elif ( TEAM_NUMBER == 5858 )
const char ACADIA_IP[] = "10.58.58.255";
#elif ( TEAM_NUMBER == 0 )
const char ACADIA_IP[] = "192.168.1.255";
#endif
*/

const char *target_mode_names[] = { "HighGoal", "LowGoal", 
	"FuelOnGround", "Hopper", "Rope", "GearPin",
	"Feeder", "GearOnGround" };

// define header values
const int ERROR_HEADER=1;
const int ERROR_REQUEST_HEADER=2;
const int IMAGE_HEADER=3;
const int IMAGE_REQUEST_HEADER=4;
const int GIMBAL_REQUEST_HEADER=5;
const int TARGET_MODE_HEADER=6;
const int INITIALIZATION_HEADER=7;

const int HSV_TYPE=0;
const int RAW_TYPE=1;
const int BINARY_TYPE=2;
const int GRAY_TYPE=3;

const int CROPPED=1;
const int NON_CROPPED=0;

const int HIGH_FUEL_TARGET=0;
const int LOW_FUEL_TARGET=1;
const int FUEL_ON_GROUND_TARGET=2;
const int HOPPER_TARGET=3;
const int ROPE_TARGET=4;
const int GEAR_PIN_TARGET=5;
const int FEEDER_TARGET=6;
const int GEAR_ON_GROUND_TARGET=7;
const int TOTAL_TARGETS=GEAR_ON_GROUND_TARGET+1;

const double logTimeInterval=2.0; // sec

const int screen_width=640; // pixels
const int screen_widthO2=screen_width/2; // pixels
const int screen_height=480; // pixels
const int screen_heightO2=screen_width/2; // pixels

const int RED=1;
const int BLUE=2;
const char *colors[] = {"", "Red", "Blue" };

// Target Mode message from Laptop
struct TargetModeMessage {
	unsigned char header;
	unsigned char mode; // see const targets including HIGH_FUEL_TARGET
	unsigned char hopper_side; // 0-left, 1-right
};

// broadcast from Acadia
struct ErrorMessage {
	unsigned char header;
	unsigned char range; // truncated inches
	unsigned char confidence; // seconds past 1970
	unsigned char target_mode; // ???? new
	short horizontal_error; // pixels
};

// broadcast from the Driver Station Laptop
struct InitializationMessage {
	unsigned char header;
	unsigned char alliance_color;
	unsigned char match_time; // int seconds
};

// broadcast from Driver Station Laptop
struct ErrorRequestMessage {
	unsigned char header;
	unsigned char mode; // 0 once, 1-as faster as possible, 2-stop sending
};

// broadcast from Acadia
struct ImageMessage {
	unsigned char header;
	unsigned char framesPerSecond; // whole number 0-30
	unsigned char alliance_color;
	short cg_x; // pixels from the center of the screen of where I am assuming center of track
	short cg_y;
	unsigned int length; // pixels
	unsigned char buffer[1];
	//unsigned int *ptr; // faster than the memcpy
};

// broadcast from Laptop
struct ImageRequestMessage {
	unsigned char header;
	unsigned char mode; // 0-binary, 1-HSV, 2-RGB, 3-Grey Image
	unsigned char cropped; // 0-no cropped image
	unsigned char freq; // seconds between messages
};

/* for Robot Code
// broadcast from RoboRIO
struct FuelResevoirMessage {
	unsigned char header;
	unsigned char level;
};
*/

