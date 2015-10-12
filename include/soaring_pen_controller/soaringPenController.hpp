#pragma once

#include<mutex>
#include<thread>
#include<chrono>

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "zmq.hpp"
#include "aruco.h"

#include "tagTrackingInfo.hpp"
#include "ARDroneEnums.hpp"
#include "command.hpp"
#include "fPoint.hpp"
#include "utilityFunctions.hpp"
#include "linearPath.hpp"

#include "SOMException.hpp"
#include "SOMScopeGuard.hpp"
#include "Poco/Timestamp.h"


#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <ardrone_autonomy/vector31.h>
#include <ardrone_autonomy/vector21.h>
#include <ardrone_autonomy/matrix33.h>

#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/LedAnim.h>
#include <ardrone_autonomy/FlightAnim.h>
#include <ardrone_autonomy/RecordEnable.h>
#include <ardrone_autonomy/Navdata.h>

#include <queue>
#include <condition_variable>
#include <chrono>
#include "gui_command.pb.h"
#include "controller_status_update.pb.h"
#include "clear_command_queue_command.pb.h"
#include "emergency_stop_command.pb.h"
#include "follow_path_command.pb.h"
#include "set_flight_animation_command.pb.h"
#include "set_led_animation_command.pb.h"
#include "set_target_altitude_command.pb.h"
#include "takeoff_command.pb.h"
#include "wait_command.pb.h"
#include "landing_command.pb.h"



//Define the size of the camera image to try to get
const int CAMERA_IMAGE_WIDTH = 1280;
const int CAMERA_IMAGE_HEIGHT = 720;

const double CAMERA_IMAGE_DIAGONAL_SIZE = sqrt(pow(CAMERA_IMAGE_WIDTH,2.0) + pow(CAMERA_IMAGE_HEIGHT,2.0));
const double HARRIS_THRESHOLD_PARAMETER_1 = 7.0;
const double HARRIS_THRESHOLD_PARAMETER_2 = 7.0;
const int HIGH_PRIORITY_COMMAND_CONSTANT = 10;
const double PATH_TRANSVERSAL_SPEED = 0.01; //How long to spend transversing one "path" unit in seconds

//This defines how long to wait for a marker sighting when in a mode reliant on state estimation before automatically landing
const double SECONDS_TO_WAIT_FOR_DRONE_MARKER_BEFORE_LANDING = 0.3;

namespace soaringPen
{


/**
This object subscribes to a set of ROS information streams that are available regarding a AR drone (some of which may be aggregators of other streams).
*/
class soaringPenController
{
public:
/**
This function initializes the controller, creates the associated interfaces and starts the associated threads.
@param inputCameraDeviceNumber: The opencv device number that the downward facing camera tracking the drone is connected to
@param inputMarkerIDNumber: The number associated with the aruco marker placed on the drone (should be 0 to 1023)
@param inputLandingPadMarkerIDNumber: The number associated with the aruco marker on the landing pad (should be 0 to 1023)
@param inputCameraIntrinsicsFilePath: The path to the file to load the camera calibration data from
@param inputHeightOfCameraFromFloor: The height of the overhead camera from the floor in meters
@param inputMarkerSizeInMeters: The marker size to pass to aruco
@param inputGUICommandInterfacePortNumber: The port to create the ZMQ pair interface used to talk with the user interface with
@param inputVideoPublishingPortNumber: The port to create the ZMQ PUB interface to publish video on


@exceptions: This function can throw exceptions
*/
soaringPenController(int inputCameraDeviceNumber, int inputDroneMarkerIDNumber, int inputLandingPadMarkerIDNumber, const std::string &inputCameraIntrinsicsFilePath, double inputHeightOfCameraFromFloor, double inputMarkerSizeInMeters, int inputGUICommandInterfacePortNumber, int inputVideoPublishingPortNumber);



/**
This function cleans up the object and waits for any threads the object spawned to return.
*/
~soaringPenController();


//Only used as callback

/**
This function is used as a callback to handle navdata info.
@param inputMessage: The legacy nav-data message to handle 
*/
void handleNavData(const ardrone_autonomy::Navdata::ConstPtr &inputMessage);

private:
/**
This function is called to update the local cache of navdata information based on the given message.
@param inputMessage: The navdata message to update the cache with
*/
void updateNavdataCache(const ardrone_autonomy::Navdata::ConstPtr &inputMessage);

/**
This function sends a message to tell the drone to takeoff using its built in takeoff sequence
@exceptions: This function can throw exceptions
*/
void activateTakeoffSequence();

/**
This function sends a message to tell the drone to land using its built in landing sequence
@exceptions: This function can throw exceptions
*/
void activateLandingSequence();

/**
This function sends a message to tell the drone to active its emergency stop sequence (cutting power to the engines)
@exceptions: This function can throw exceptions
*/
void activateEmergencyStop();

/**
This function sends a message to set the linear and angular velocity of the drone
@param inputVelocityX: The X velocity of the drone
@param inputVelocityY: The Y velocity of the drone
@param inputVelocityZ: The Z velocity of the drone
@param inputRotationZ: The Z rotation rate of the drone
@exceptions: This function can throw exceptions
*/
void setVelocityAndRotation(double inputVelocityX, double inputVelocityY, double inputVelocityZ, double inputRotationZ);

/**
This function sets the linear and angular velocity of the drone to zero and enables the auto-hover mode to try to maintain its position
@exceptions: This function can throw exceptions
*/
void enableAutoHover();

/**
This function either sets the active camera to the front facing one or the bottom one
@param inputSetCameraFront: True if the front camera should be set active, false if the bottom camera should be

@exceptions: This function can throw exceptions
*/
void setCameraFront(bool inputSetCameraFront);

/**
This function will trigger an LED animation sequence on the quadcopter
@param inputAnimationType: The type of the animation
@param inputFrequency: The frequency of the animation (if blinking), in hertz
@param inputDuration: The duration of the animation in seconds

@exceptions: This function can throw exceptions
*/
void activateLEDAnimationSequence(LEDAnimationType inputAnimationType, double inputFrequency, int inputDuration);

/**
This function will trigger a flight animation
@param inputFlightAnimationType: What type of flight animation to perform

@exceptions: This function can throw exceptions
*/
void activateFlightAnimation(flightAnimationType inputFlightAnimationType);

/**
This function causes the drone to recalibrate its rotations using the assumption that it is on a flat service.  Don't use it when it is not on a flat surface.
@exceptions: This function can throw exceptions
*/
void calibrateFlatTrim();

/**
This function causes the drone to start recording to its USB stick (if it has one) or stop recording.
@param inputStartRecording: Set to true if the drone should start recording and false if it should stop
@exceptions: This function can throw exceptions
*/
void enableUSBRecording(bool inputStartRecording);

/**
This command checks the conditions associated with the current command type and returns if they have been completed.  This function will always return true if the current command is invalid.
@return: True if the current command has been completed or is invalid, false otherwise
*/
bool checkIfCurrentCommandIsCompleted();

/**
This function returns true if the ARDrone has achieved the hovering state.
@return: True if the hovering state has been achieved
*/
bool checkIfHoveringStateAchieved();

/**
This function returns if the ARDrone has achieved the landed state.
@return: True if the landed state has been achieved
*/
bool checkIfLandedStateAchieved();

/**
This function checks if a tag has been spotted.
@return: True if there is one or more tags in the navdata cache
*/
bool checkIfTagSpotted();

/**
This function checks if the target altitude is reached.
@param inputNumberOfMillimetersToTarget: The wiggle room to match the target height (+- this amount from the target).  It is normally 10 mm.
*/
bool checkIfAltitudeReached(int inputNumberOfMillimetersToTarget = 10);

/**
This function replaces the current command if there is a higher priority one in the queue or the conditions associated with the command have been met.
@return: true if the current command has been changed, false otherwise
*/
bool updateCurrentCommand();

/**
This function takes the appropriate actions for control given the current commands in the queue and data state.  It calls lower level functions to send out the appropriate commands.
*/
void processCurrentCommandsForUpdateCycle();

/**
This function adjusts and enables/disables low level behavior depending on the current command.
@return: true if the command has been completed and false otherwise (allowing multiple commands to be executed in a single update cycle if they are instant).

@exceptions: This function can throw exceptions
*/
bool adjustBehavior();

/**
This function takes care of low level behavior that depends on the specific state variables (such as reaching the desired altitude and orientation).

@exceptions: This function can throw exceptions
*/
void handleLowLevelBehavior();

/**
This function takes video frames from the camera device, publishes them and updates the aruco marker based state estimation information for use by the ROS callbacks.  This function is typically called in a seperate thread.
*/
void videoProcessingAndPublishingFunction();

/**
This function repeatedly calls ros::spinOnce until the shouldExitFlag is set (usually by the object destructor).
*/
void initializeAndRunSpinThread();


//Owned by the videoProcessingAndPublishingThread
cv::VideoCapture imageSource;
cv::Mat sourceImage;
int droneMarkerIDNumber; //The ID of the aruco marker to look for
int landingPadMarkerIDNumber;
double markerSizeInMeters; //Used with aruco
double heightOfCameraFromFloor;
aruco::CameraParameters cameraIntrinsics;
aruco::MarkerDetector markerDetector;

std::mutex arucoDataMutex; //Prevent modification by more than one thread at a time
int framesSinceDroneDetected = 9999999; //The number of frames since the drone marker was seen (should be 0 in normal operation)
fPoint droneCenter; //Center in the camera image
fPoint droneXAxis; //X axis in the camera image
fPoint droneYAxis; //Y axis in the camera image
bool landingAreaHasBeenDetected = false;
fPoint landingAreaCenter; //Center in the camera image
fPoint landingAreaXAxis; //X axis in the camera image
fPoint landingAreaYAxis; //Y axis in the camera image

int commandCounter = 0; //Used to number commands as they enter the queue
std::priority_queue<command> commandQueue;
int completedCommandNumber = -1;
command currentCommand; //The command we are currently executing, set at a command number of -1 at initialization
Poco::Timestamp timeCurrentCommandStarted; //Updated whenever a new command is started

linearPath path;
fPoint pathLocationBuffer;
fPoint pathTargetPointInLocalRelativeImageCoordinates; //The path point the drone is trying to get to in relative image coordinates relative to the drone
fPoint pathTargetPointDerivativeLocalInRelativeImageCoordinates; //The derivative of the point the drone is attempting to go for the drone's coordinate system
fPoint targetOrientationInLocalCoordinates; //A vector pointing to where the drone is suppose to be pointing, in the drone's local coordinate system
bool shutdownControlEngine;
bool controlEngineIsDisabled;
bool onTheGroundWithMotorsOff;
bool takingOff;
bool landing;
bool emergencyStopTriggered;
bool maintainAltitude;
bool homeInOnTag;
bool matchTagOrientation;
bool maintainQRCodeDefinedPosition;
bool maintainQRCodeDefinedOrientation;
double targetAltitude; //The altitude to maintain in mm
double targetAltitudeITerm;
double xHeading; //The current velocity settings of the drone
double yHeading; 
double currentAngularVelocitySetting; //The setting of the current velocity
fPoint pathFollowingITerm;
bool currentlyWaiting;

enum droneCurrentState state;  //The current state of the drone
double batteryPercent;  //Percentage of the drone's battery remaining
double rotationX; //Left/Right tilt in degrees
double rotationY; //Forward/backward tilt in degrees
double rotationZ; //Turn rotation estimate
double magneticX; //Magnetic reading in X
double magneticY; //Magnetic reading in Y
double magneticZ; //Magnetic reading in Z
double pressure;  //Pressure sensed by barometer (Pa)
double temperature; //Temperature reading
double windSpeed;  //Estimated wind speed
double windAngle; //Estimated wind angle
double windCompensationAngle;  //?
double altitude;   //Estimated altitude (mm)
double motorPWM[4];//Current PWM values for the motors
double velocityX;  //Current estimated X velocity (mm/s)
double velocityY;  //Current estimated Y velocity (mm/s)
double velocityZ;  //Current estimated Z velocity (mm/s)
double accelerationX;  //Current estimated X acceleration
double accelerationY;  //Current estimated Y acceleration
double accelerationZ;  //Current estimated Z acceleration
std::vector<tagTrackingInfo> trackedTags; //Information about any oriented roundel tags in the field of view
std::chrono::time_point<std::chrono::high_resolution_clock> navdataUpdateTime; //The timestamp of when the navdata update was received



ros::NodeHandle nodeHandle;

bool shouldExitFlag; //This flag indicates if thread that is calling spinOnce should exit
std::unique_ptr<std::thread> spinThread;
std::unique_ptr<std::thread> videoProcessingAndPublishingThread;

std::unique_ptr<zmq::context_t> context;
std::unique_ptr<zmq::socket_t> commandReceiver; //A ZMQ PAIR socket which expects gui_command messages and sends controller_status_update messages
std::unique_ptr<zmq::socket_t> videoPublisher; //A ZMQ PUB socket which publishs JPeg images (using opencv for serialization) from the camera device

ros::Subscriber navDataSubscriber; 
ros::Subscriber videoSubscriber; 

ros::Publisher takeOffPublisher;
ros::Publisher landingPublisher;
ros::Publisher emergencyStopPublisher;
ros::Publisher directionAndSpeedPublisher;

ros::ServiceClient cameraControlClient;
ros::ServiceClient setLEDAnimationClient;
ros::ServiceClient setFlightAnimationClient;
ros::ServiceClient calibrateFlatTrimClient;
ros::ServiceClient setUSBRecordingClient;

ros::Publisher QRCodeStateInfoPublisher;
ros::Publisher altitudeControlInfoPublisher;
ros::Publisher QRCodeGoToPointControlInfoPublisher;
ros::Publisher QRCodeOrientationControlInfoPublisher;
ros::Publisher commandProcessingInfoPublisher;
};




/**
This point takes a marker and set of camera intrinsics that were used to generate the marker and stores the centerpoint of the marker, as well as its x axis and y axis in the image.
@param inputMarker: The marker to process to establish position
@param inputCameraIntrinsics: The intrinsics of the camera used with marker
@param inputCenterPointBuffer: The variable to store the resulting center point in
@param inputXAxisBuffer: The variable to store the resulting 2d X axis in
@param inputYAxisBuffer: The variable to store the resulting 2d Y axis in
*/
void getCenterPointAndAxis(aruco::Marker &inputMarker, aruco::CameraParameters &inputCameraIntrinsics, soaringPen::fPoint &inputCenterPointBuffer, soaringPen::fPoint &inputXAxisBuffer, soaringPen::fPoint &inputYAxisBuffer);

}
