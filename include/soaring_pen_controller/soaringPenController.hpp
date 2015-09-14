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

#include "SOMException.hpp"
#include "SOMScopeGuard.hpp"

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "ardrone_command/serialized_ardrone_command.h"
#include "ardrone_command/altitude_control_state.h"
#include "ardrone_command/qr_code_state_info.h"
#include "ardrone_command/qr_go_to_point_control_info.h"
#include "ardrone_command/qr_orientation_control_info.h"
#include "ardrone_command/command_status_info.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

//This defines how long to wait for a QR code sighting when in a mode reliant on QR code state estimation before automatically landing
#define SECONDS_TO_WAIT_FOR_QR_CODE_BEFORE_LANDING 1

//This defines how long to wait (in microseconds) for a QR code before considering the drone to be experiencing "high latency" and having it simply hover for a little while till things mellow out or it considers itself to have lost tracking
#define HIGH_LATENCY_WATER_MARK .3

//Topics that the drone publishes for public consumption
#define ALTITUDE_CONTROL_PUBLISHER_STRING "/ardrone_command/altitude_control"
#define COMMAND_PROCESSING_INFO_PUBLISHER_STRING "/ardrone_command/command_processing"


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
@param inputGUICommandInterfacePortNumber: The port to create the ZMQ pair interface used to talk with the user interface with
@param inputVideoPublishingPortNumber: The port to create the ZMQ PUB interface to publish video on

@exceptions: This function can throw exceptions
*/
soaringPenController(int inputCameraDeviceNumber, int inputMarkerIDNumber, int inputGUICommandInterfacePortNumber, int inputVideoPublishingPortNumber);



/**
This function cleans up the object and waits for any threads the object spawned to return.
*/
~soaringPenController();


friend void initializeAndRunSpinThread(soaringPenController *inputsoaringPenController);

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
This function takes the appropriate actions for control given the current commands in the queue and data state.  It calls lower level functions to send out the appropriate commands.
*/
void processCurrentCommandsForUpdateCycle();

/**
This function adjusts and enables/disables low level behavior depending on the given command.
@param inputCommand: The command to execute.
@return: true if the command has been completed and false otherwise (allowing multiple commands to be executed in a single update cycle if they are instant).

@exceptions: This function can throw exceptions
*/
bool adjustBehavior(const command &inputCommand);

/**
This function takes care of low level behavior that depends on the specific state variables (such as reaching the desired altitude and orientation).

@exceptions: This function can throw exceptions
*/
void handleLowLevelBehavior();

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
int commandCounter; //How many commands have been completed
int lastPublishedCommandCount; //Number of last command that the object published about
double targetAltitude; //The altitude to maintain in mm
double targetAltitudeITerm;
double xHeading; //The current velocity settings of the drone
double yHeading; 
double xCoordinateITerm;
double yCoordinateITerm;
double currentAngularVelocitySetting; //The setting of the current velocity
std::vector<double> targetXYZCoordinate; //The target point to reach, if any
double QRTargetXITerm;
double QRTargetYITerm;
std::string targetXYZCoordinateQRCodeIdentifier; //The identifier of the QR code that defines the coordinate system for position holding
std::string targetOrientationQRCodeIdentifier; //The identifier of the QR code that we are trying to maintain orientation relative to (if any)
std::string QRCodeToSpotIdentifier; //Identifier of QR code to try to spot
bool currentlyWaiting;
std::chrono::time_point<std::chrono::high_resolution_clock> waitFinishTime; //When any current wait command is due to expire

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

bool spinThreadExitFlag; //This flag indicates if thread that is calling spinOnce should exit
std::unique_ptr<std::thread> spinThread;


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
This function repeatedly calls ros::spinOnce until the spinThreadExitFlag in the given object is set (usually by the object destructor.  It is usually run in a seperate thread.
@param inputsoaringPenController: The node this function call is associated
*/
void initializeAndRunSpinThread(soaringPenController *inputsoaringPenController);

}
