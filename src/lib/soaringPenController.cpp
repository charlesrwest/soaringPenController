#include "soaringPenController.hpp"



using namespace soaringPen;

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
soaringPenController::soaringPenController(int inputCameraDeviceNumber, int inputDroneMarkerIDNumber, int inputLandingPadMarkerIDNumber, const std::string &inputCameraIntrinsicsFilePath, double inputHeightOfCameraFromFloor, double inputMarkerSizeInMeters, int inputGUICommandInterfacePortNumber, int inputVideoPublishingPortNumber)
{
shouldExitFlag = false;
controlEngineIsDisabled = false;
shutdownControlEngine = false;
currentlyWaiting = false;
markerSizeInMeters = inputMarkerSizeInMeters;
droneMarkerIDNumber = inputDroneMarkerIDNumber;
landingPadMarkerIDNumber = inputLandingPadMarkerIDNumber;
heightOfCameraFromFloor = inputHeightOfCameraFromFloor;

//Open image source
if(imageSource.open(inputCameraDeviceNumber) != true)
{
throw SOMException("Unable to open given camera device number\n", INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

//imageSource.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('Y', 'U', 'Y', 'V') );
//imageSource.set(CV_CAP_PROP_FPS,10);
imageSource.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_IMAGE_WIDTH);
imageSource.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_IMAGE_HEIGHT);

//Get first image (used for automatic dimension detection)
SOM_TRY
imageSource >> sourceImage;
SOM_CATCH("Error retrieving first image\n")

onTheGroundWithMotorsOff = true;
takingOff = false;
landing = false;
emergencyStopTriggered = false;
maintainAltitude = false;
maintainQRCodeDefinedPosition = false;
maintainQRCodeDefinedOrientation = false;
targetAltitude = 500.0; //The altitude to maintain in mm
xHeading = 0.0; 
yHeading = 0.0; 
currentAngularVelocitySetting = 0.0; //The setting of the current velocity

//Create ZMQ context
SOM_TRY
context.reset(new zmq::context_t);
SOM_CATCH("Error initializing ZMQ context\n")

//Initialize and bind command receiver socket
SOM_TRY //Initialize
commandReceiver.reset(new zmq::socket_t(*(context), ZMQ_PAIR));
SOM_CATCH("Error initializing command socket\n")

SOM_TRY //Bind
std::string bindingAddress = "tcp://*:"+ std::to_string(inputGUICommandInterfacePortNumber);
commandReceiver->bind(bindingAddress.c_str());
SOM_CATCH("Error binding command receiver\n")

SOM_TRY //Initialize
videoPublisher.reset(new zmq::socket_t(*(context), ZMQ_PUB));
SOM_CATCH("Error initializing video sharing socket\n")

SOM_TRY //Bind
std::string bindingAddress = "tcp://*:"+ std::to_string(inputVideoPublishingPortNumber);
videoPublisher->bind(bindingAddress.c_str());
SOM_CATCH("Error binding video publisher\n")

//Setup aruco
//Read camera parameters
SOM_TRY
cameraIntrinsics.readFromXMLFile(inputCameraIntrinsicsFilePath);
SOM_CATCH("Error, unable to read camera intrinsics from " + inputCameraIntrinsicsFilePath + "\n")

//Configure marker detector
markerDetector.setThresholdParams(HARRIS_THRESHOLD_PARAMETER_1, HARRIS_THRESHOLD_PARAMETER_2);
markerDetector.setThresholdParamRange(2,0); //Not sure what this does

//Automatically resize the camera calibration object to match the current image
SOM_TRY
cameraIntrinsics.resize(sourceImage.size());
SOM_CATCH("Error resizing camera parameters\n") 

//Subscribe to the nav-data topic with a buffer size of 1000
navDataSubscriber = nodeHandle.subscribe("ardrone/navdata", 1000, &soaringPenController::handleNavData, this); 

//Subscribe to the video from the AR drone with a buffer size of 5
//videoSubscriber = nodeHandle.subscribe("ardrone/front/image_raw", 5, &soaringPenController::handleImageUpdate, this); 

//Create publisher to control takeoff
takeOffPublisher = nodeHandle.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);

//Create publisher to control landing
landingPublisher = nodeHandle.advertise<std_msgs::Empty>("/ardrone/land", 1000);

//Create publisher to control emergency stop
emergencyStopPublisher = nodeHandle.advertise<std_msgs::Empty>("/ardrone/reset", 1000);

//Create publisher to control heading and speed
directionAndSpeedPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

//Create client to control which camera is active
cameraControlClient = nodeHandle.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/setcamchannel");

//Create client to activate LED animations
setLEDAnimationClient = nodeHandle.serviceClient<ardrone_autonomy::LedAnim>("/ardrone/setledanimation");

//Create client to activate flight animations
setFlightAnimationClient = nodeHandle.serviceClient<ardrone_autonomy::FlightAnim>("/ardrone/setflightanimation");

//Create client to allow flat trim calibration
calibrateFlatTrimClient = nodeHandle.serviceClient<std_srvs::Empty>("/ardrone/flattrim");

//Create client to enable/disable USB recording
setUSBRecordingClient = nodeHandle.serviceClient<ardrone_autonomy::RecordEnable>("/ardrone/setrecord");

//Sleep a little while to allow subscribers to connect
while(takeOffPublisher.getNumSubscribers() < 1)
{
sleep(1);
}

while(landingPublisher.getNumSubscribers() < 1)
{
sleep(1);
}

while(emergencyStopPublisher.getNumSubscribers() < 1)
{
sleep(1);
}

while(directionAndSpeedPublisher.getNumSubscribers() < 1)
{
sleep(1);
}

//Create seperate thread to call spin
SOM_TRY
spinThread.reset(new std::thread(&soaringPenController::initializeAndRunSpinThread, this));
SOM_CATCH("Error starting ROS message processing thread\n")


SOM_TRY
videoProcessingAndPublishingThread.reset(new std::thread(&soaringPenController::videoProcessingAndPublishingFunction, this));
SOM_CATCH("Error starting video processing/publishing thread\n")

printf("Initialization complete\n");
}


/**
This function is called to update the local cache of navdata information based on the given message.
@param inputMessage: The navdata message to update the cache with
*/
void soaringPenController::updateNavdataCache(const ardrone_autonomy::Navdata::ConstPtr &inputMessage)
{
batteryPercent = inputMessage->batteryPercent;

switch(inputMessage->state)
{
case 0:
state = UNKNOWN_STATE;
break;

case 1:
state = INITIALIZED;
break;

case 2:
state = LANDED;
break;

case 3:
case 7:
state = FLYING;
break;

case 4:
state = HOVERING;
break;

case 5:
state = TEST;
break;

case 6:
state = TAKING_OFF;
break;

case 8:
state = LANDING;
break;

case 9:
state = LOOPING;
break;
}

rotationX = inputMessage->rotX;
rotationY = inputMessage->rotY;
rotationZ = inputMessage->rotZ;
magneticX = inputMessage->magX;
magneticY = inputMessage->magY;
magneticZ = inputMessage->magZ;
pressure  = inputMessage->pressure;
temperature = inputMessage->temp;
windSpeed = inputMessage->wind_speed;
windAngle = inputMessage->wind_angle;
windCompensationAngle = inputMessage->wind_comp_angle;
altitude  = inputMessage->altd;
motorPWM[0] = inputMessage->motor1;
motorPWM[1] = inputMessage->motor2;
motorPWM[2] = inputMessage->motor3;
motorPWM[3] = inputMessage->motor4;
velocityX = inputMessage->vx;
velocityY = -inputMessage->vy; //Make right positive
velocityZ = inputMessage->vz;
accelerationX = inputMessage->ax;
accelerationY = inputMessage->ay;
accelerationZ = inputMessage->az;

//Set tag info
trackedTags.clear();
for(int i=0; i < inputMessage->tags_count; i++)
{
trackedTags.push_back(tagTrackingInfo(inputMessage->tags_xc[i], inputMessage->tags_yc[i], inputMessage->tags_width[i], inputMessage->tags_height[i], inputMessage->tags_orientation[i], inputMessage->tags_distance[i]));
}

//Set when the update was received
navdataUpdateTime = std::chrono::high_resolution_clock::now();
}

/**
This function sends a message to tell the drone to takeoff using its built in takeoff sequence
@exceptions: This function can throw exceptions
*/
void soaringPenController::activateTakeoffSequence()
{
std_msgs::Empty emptyMessage;
takeOffPublisher.publish(emptyMessage);
}

/**
This function sends a message to tell the drone to land using its built in landing sequence
@exceptions: This function can throw exceptions
*/
void soaringPenController::activateLandingSequence()
{
std_msgs::Empty emptyMessage;
landingPublisher.publish(emptyMessage);
}

/**
This function sends a message to tell the drone to active its emergency stop sequence (cutting power to the engines)
@exceptions: This function can throw exceptions
*/
void soaringPenController::activateEmergencyStop()
{
std_msgs::Empty emptyMessage;
emergencyStopPublisher.publish(emptyMessage);
}

/**
This function sends a message to set the linear and angular velocity of the drone
@param inputVelocityX: The X velocity of the drone
@param inputVelocityY: The Y velocity of the drone
@param inputVelocityZ: The Z velocity of the drone
@param inputRotationZ: The Z rotation rate of the drone
@exceptions: This function can throw exceptions
*/
void soaringPenController::setVelocityAndRotation(double inputVelocityX, double inputVelocityY, double inputVelocityZ, double inputRotationZ)
{
geometry_msgs::Twist twistMessage;

twistMessage.linear.x = inputVelocityX;
twistMessage.linear.y = -inputVelocityY; //Make right positive
twistMessage.linear.z = inputVelocityZ;

twistMessage.angular.x = 1.0;
twistMessage.angular.y = 1.0;
twistMessage.angular.z = inputRotationZ;

directionAndSpeedPublisher.publish(twistMessage);
}

/**
This function sets the linear and angular velocity of the drone to zero and enables the auto-hover mode to try to maintain its position
@exceptions: This function can throw exceptions
*/
void soaringPenController::enableAutoHover()
{
geometry_msgs::Twist twistMessage;

twistMessage.linear.x = 0.0;
twistMessage.linear.y = 0.0;
twistMessage.linear.z = 0.0;

twistMessage.angular.x = 0.0;
twistMessage.angular.y = 0.0;
twistMessage.angular.z = 0.0;

directionAndSpeedPublisher.publish(twistMessage);
}

/**
This function either sets the active camera to the front facing one or the bottom one
@param inputSetCameraFront: True if the front camera should be set active, false if the bottom camera should be

@exceptions: This function can throw exceptions
*/
void soaringPenController::setCameraFront(bool inputSetCameraFront)
{
ardrone_autonomy::CamSelect requestMessage;
if(inputSetCameraFront == true)
{
requestMessage.request.channel = 0;
}
else
{
requestMessage.request.channel = 1;
}

if(cameraControlClient.call(requestMessage))
{
//Request succeeded
}
else
{
}

}

/**
This function will trigger an LED animation sequence on the quadcopter
@param inputAnimationType: The type of the animation
@param inputFrequency: The frequency of the animation (if blinking), in hertz
@param inputDuration: The duration of the animation in seconds

@exceptions: This function can throw exceptions
*/
void soaringPenController::activateLEDAnimationSequence(LEDAnimationType inputAnimationType, double inputFrequency, int inputDuration)
{

ardrone_autonomy::LedAnim requestMessage;

requestMessage.request.type = (int) inputAnimationType;

requestMessage.request.freq = inputFrequency;
requestMessage.request.duration = inputDuration;

if(setLEDAnimationClient.call(requestMessage))
{
//Request succeeded
}
else
{
}

}

/**
This function will trigger a flight animation
@param inputFlightAnimationType: What type of flight animation to perform

@exceptions: This function can throw exceptions
*/
void soaringPenController::activateFlightAnimation(flightAnimationType inputFlightAnimationType)
{
ardrone_autonomy::FlightAnim requestMessage;

requestMessage.request.type = (int) inputFlightAnimationType;
requestMessage.request.duration = 0;

if(setFlightAnimationClient.call(requestMessage))
{
//Request succeeded
}
else
{
}

}

/**
This function causes the drone to recalibrate its rotations using the assumption that it is on a flat service.  Don't use it when it is not on a flat surface.
@exceptions: This function can throw exceptions
*/
void soaringPenController::calibrateFlatTrim()
{
std_srvs::Empty requestMessage;


if(calibrateFlatTrimClient.call(requestMessage))
{
//Request succeeded
}
else
{
}
}

/**
This function causes the drone to start recording to its USB stick (if it has one) or stop recording.
@param inputStartRecording: Set to true if the drone should start recording and false if it should stop
@exceptions: This function can throw exceptions
*/
void soaringPenController::enableUSBRecording(bool inputStartRecording)
{
ardrone_autonomy::RecordEnable requestMessage;
requestMessage.request.enable = inputStartRecording;

if(setUSBRecordingClient.call(requestMessage))
{
//Request succeeded
}
else
{
}

}

/**
This function is used as a callback to handle nav-data.
@param inputMessage: The  nav-data message to handle 
*/
void soaringPenController::handleNavData(const ardrone_autonomy::Navdata::ConstPtr &inputMessage)
{
//Update navdata cache
updateNavdataCache(inputMessage);

//Send control messages for this cycle and process commands
SOM_TRY
processCurrentCommandsForUpdateCycle();
SOM_CATCH("Error adjusting/sending commands for commands/behavior")

//Send update to GUI
fPoint droneCenterBuffer;
fPoint landingAreaCenterBuffer;
{
std::lock_guard<std::mutex> lock(arucoDataMutex); //Lock mutex to access state data
droneCenterBuffer = droneCenter;
landingAreaCenterBuffer = landingAreaCenter;
}

//Compose message
controller_status_update statusUpdate;

statusUpdate.set_battery_status(batteryPercent);
statusUpdate.set_drone_x_velocity(velocityX);
statusUpdate.set_drone_y_velocity(velocityY);
statusUpdate.set_drone_x_position(droneCenterBuffer.val[0]);
statusUpdate.set_drone_y_position(droneCenterBuffer.val[1]);
statusUpdate.set_landing_area_x_position(landingAreaCenter.val[0]);
statusUpdate.set_landing_area_y_position(landingAreaCenter.val[1]);
statusUpdate.set_completed_command_number(completedCommandNumber);

//Send message
SOM_TRY
sendProtobufMessage(*commandReceiver, statusUpdate);
SOM_CATCH("Error serializing/sending protobuf message\n")
}



/**
This function cleans up the object and waits for any threads the object spawned to return.
*/
soaringPenController::~soaringPenController()
{
shouldExitFlag = true;
spinThread->join();
}

/**
This command checks the conditions associated with the current command type and returns if they have been completed.  This function will always return true if the current command is invalid.
@return: True if the current command has been completed or is invalid, false otherwise
*/
bool soaringPenController::checkIfCurrentCommandIsCompleted()
{
//TODO: Finish this function
if(currentCommand.commandNumber == -1)
{
return true;
}

if(currentCommand.HasExtension(clear_command_queue_command::clear_command_queue_command_field))
{
return true;
}

if(currentCommand.HasExtension(emergency_stop_command::emergency_stop_command_field))
{
return true;
}

if(currentCommand.HasExtension(follow_path_command::follow_path_command_field))
{
Poco::Timestamp currentTime;
double timeDifference =  (currentTime - timeCurrentCommandStarted)/1000000.0;

if(path.pathLength < PATH_TRANSVERSAL_SPEED*timeDifference)
{
path = linearPath();
printf("Path completed\n");
return true; //Path already completed
}
else
{
return false;
}
}

if(currentCommand.HasExtension(set_flight_animation_command::set_flight_animation_command_field))
{
return true; //Instant commands return true
}

if(currentCommand.HasExtension(set_led_animation_command::set_led_animation_command_field))
{
return true; //Instant commands return true
}

if(currentCommand.HasExtension(set_target_altitude_command::set_target_altitude_command_field))
{
return true; //Instant commands return true
}

if(currentCommand.HasExtension(takeoff_command::takeoff_command_field))
{
return true;
}

if(currentCommand.HasExtension(wait_command::wait_command_field))
{
Poco::Timestamp currentTime;
if((currentTime - timeCurrentCommandStarted)/1000000.0 >= currentCommand.GetExtension(wait_command::wait_command_field).wait_duration())
{
return true;
}

return false;
}

if(currentCommand.HasExtension(landing_command::landing_command_field))
{
return true;
}

return true;
}

/**
This function returns true if the ARDrone has achieved the hovering state.
@return: True if the hovering state has been achieved
*/
bool soaringPenController::checkIfHoveringStateAchieved()
{
return state == HOVERING || state == FLYING;
}

/**
This function returns if the ARDrone has achieved the landed state.
@return: True if the landed state has been achieved
*/
bool soaringPenController::checkIfLandedStateAchieved()
{
return state == LANDED;
}

/**
This function checks if a tag has been spotted.
@return: True if there is one or more tags in the navdata cache
*/
bool soaringPenController::checkIfTagSpotted()
{
return trackedTags.size() > 0;
}


/**
This function checks if the target altitude is reached.
@param inputNumberOfMillimetersToTarget: The wiggle room to match the target height (+- this amount from the target).  It is normally 10 mm.
*/
bool soaringPenController::checkIfAltitudeReached(int inputNumberOfMillimetersToTarget)
{
return fabs(altitude - targetAltitude) < inputNumberOfMillimetersToTarget;
}

/**
This function replaces the current command if there is a higher priority one in the queue or the conditions associated with the command have been met.
@return: true if the current command has been changed, false otherwise
*/
bool soaringPenController::updateCurrentCommand()
{
if(commandQueue.size() > 0)
{
if(checkIfCurrentCommandIsCompleted() || currentCommand.priority() < commandQueue.top().priority())
{
printf("Moving to next command\n");
currentCommand = commandQueue.top();
commandQueue.pop();
timeCurrentCommandStarted = Poco::Timestamp(); //Set command started time to current time since a new command was loaded
return true;
}
}

printf("Command queue size: %ld\n", commandQueue.size());

if(commandQueue.size() == 0 && checkIfCurrentCommandIsCompleted())
{
currentCommand.commandNumber = -1;
}

return false;
}




/**
This function takes the appropriate actions for control given the current commands in the queue and data state.  It calls lower level functions to send out the appropriate commands.
*/
void soaringPenController::processCurrentCommandsForUpdateCycle()
{
try
{
while(true)
{
//Update the current command in case it has been completed
updateCurrentCommand();

if(currentCommand.commandNumber == -1 && onTheGroundWithMotorsOff)
{
return; //Current command is invalid, but we are on the ground so it is OK
}


if(currentCommand.commandNumber == -1 && !onTheGroundWithMotorsOff)
{//Flying, but without commands... Land immediately
printf("Initiating automatic landing due to lack of commands\n");
landing_command landCommand;
command commandBuffer(commandCounter);
commandCounter++;

(*commandBuffer.MutableExtension(landing_command::landing_command_field)) = landCommand;
commandQueue.push(commandBuffer);
}


SOM_TRY
if(adjustBehavior() == false)
{ //This command will take time to complete, so exit the loop
break;
}
else
{
printf("Command completed\n");
}
SOM_CATCH("Error adjusting behavior\n")
}

//Handle low level behaviors using the settings from the adjusted behavior
printf("About to handle low level stuff\n");
SOM_TRY
handleLowLevelBehavior();
SOM_CATCH("Error, problem implementing low level behavior\n")
}
catch(const std::exception &inputException)
{
printf("Exception: %s\n", inputException.what());
}


printf("Function completed\n");
}


/**
This function adjusts and enables/disables low level behavior depending on the current command.
@return: true if the command has been completed and false otherwise (allowing multiple commands to be executed in a single update cycle if they are instant).

@exceptions: This function can throw exceptions
*/
bool soaringPenController::adjustBehavior()
{
if(currentCommand.commandNumber == -1)
{
printf("No command available\n");
return false;
}

if(currentCommand.HasExtension(clear_command_queue_command::clear_command_queue_command_field))
{
printf("Clearing command queue\n");
while(commandQueue.size() > 0)
{
commandQueue.pop();
}
currentCommand.commandNumber = -1; //Clear command completed

return true;
}

if(currentCommand.HasExtension(emergency_stop_command::emergency_stop_command_field))
{
printf("Operating on emergency stop command\n");
SOM_TRY
activateEmergencyStop();
SOM_CATCH("Error activating emergency stop\n")
emergencyStopTriggered = true;
while(commandQueue.size() > 0)
{
commandQueue.pop();
}
return checkIfCurrentCommandIsCompleted();
}

if(currentCommand.HasExtension(follow_path_command::follow_path_command_field))
{//TODO: Figure out some way of determining if this is the first time, so the path doesn't always need to be regenerated
printf("Operating on follow path command\n");
//Construct path object from command
const follow_path_command &commandBuffer = currentCommand.GetExtension(follow_path_command::follow_path_command_field);
path.points.clear();
for(int i=0; i<commandBuffer.path_x_coordinates_size() && i<commandBuffer.path_y_coordinates_size(); i++)
{
path.addPoint(fPoint(commandBuffer.path_x_coordinates(i), commandBuffer.path_y_coordinates(i)));
}

Poco::Timestamp currentTime;
double timeDifference =  (currentTime-timeCurrentCommandStarted)/1000000.0;

printf("Path location %lf\nPath Length %lf\n", PATH_TRANSVERSAL_SPEED*timeDifference,path.pathLength);

if(path.pathLength < PATH_TRANSVERSAL_SPEED*timeDifference )
{
printf("I think the path is completed\n");
return checkIfCurrentCommandIsCompleted(); //Path already completed
}

//Calculate the current path location in relative image coordinates
fPoint droneCenterBuffer;
fPoint droneXAxisBuffer;
fPoint droneYAxisBuffer;

{
std::lock_guard<std::mutex> lock(arucoDataMutex); //Lock mutex to access state data
droneCenterBuffer = droneCenter;
droneXAxisBuffer = droneXAxis;
droneYAxisBuffer = droneYAxis;
}
droneXAxisBuffer.normalize(); //Make unit vectors
droneYAxisBuffer.normalize();

printf("X axis: %lf %lf\nY axis: %lf %lf\n", droneXAxisBuffer.val[0], droneXAxisBuffer.val[1], droneYAxisBuffer.val[0], droneYAxisBuffer.val[1]);

targetOrientationInLocalCoordinates.val[0] = dot(fPoint(0.0, 1.0), droneXAxisBuffer);
targetOrientationInLocalCoordinates.val[1] = dot(fPoint(0.0, 1.0), droneYAxisBuffer);

//Compute current path location in image coordinates, then convert to local coordinates
fPoint pathLocation;
SOM_TRY
pathLocation = path.interpolate(PATH_TRANSVERSAL_SPEED*timeDifference);
SOM_CATCH("Error interpolating path\n")
fPoint pathLocationDifference = pathLocation - droneCenterBuffer;

pathLocationBuffer = pathLocation;

//Project onto the axises to get local coordinates relative to the drone
pathTargetPointInLocalRelativeImageCoordinates.val[0] = dot(pathLocationDifference, droneXAxisBuffer);
pathTargetPointInLocalRelativeImageCoordinates.val[1] = dot(pathLocationDifference, droneYAxisBuffer);

//Do the same for the derivative of the path
fPoint pathDerivative = path.derivative(PATH_TRANSVERSAL_SPEED*timeDifference);

pathTargetPointDerivativeLocalInRelativeImageCoordinates.val[0] = dot(pathDerivative, droneXAxisBuffer);
pathTargetPointDerivativeLocalInRelativeImageCoordinates.val[1] = dot(pathDerivative, droneYAxisBuffer);
return checkIfCurrentCommandIsCompleted();
}

if(currentCommand.HasExtension(set_flight_animation_command::set_flight_animation_command_field))
{
SOM_TRY
activateFlightAnimation((flightAnimationType) currentCommand.GetExtension(set_flight_animation_command::set_flight_animation_command_field).flight_type());
SOM_CATCH("Error activating flight animation\n")
return checkIfCurrentCommandIsCompleted(); //Instant commands return true
}

if(currentCommand.HasExtension(set_led_animation_command::set_led_animation_command_field))
{
const set_led_animation_command &commandBuffer = currentCommand.GetExtension(set_led_animation_command::set_led_animation_command_field);
SOM_TRY
activateLEDAnimationSequence((LEDAnimationType) commandBuffer.animation_type(), commandBuffer.frequency(), commandBuffer.duration());
SOM_CATCH("Error activating LED animation\n")
return checkIfCurrentCommandIsCompleted(); //Instant commands return true
}

if(currentCommand.HasExtension(set_target_altitude_command::set_target_altitude_command_field))
{
printf("Setting target altitude\n");
const set_target_altitude_command &commandBuffer = currentCommand.GetExtension(set_target_altitude_command::set_target_altitude_command_field);
targetAltitude = commandBuffer.target_altitude();
targetAltitudeITerm = 0.0;
maintainAltitude = true; //Enable trying to meet a set altitude
return checkIfCurrentCommandIsCompleted(); //Instant commands return true
}

if(currentCommand.HasExtension(takeoff_command::takeoff_command_field))
{
printf("Taking off\n");
SOM_TRY
activateTakeoffSequence();
onTheGroundWithMotorsOff = false;
SOM_CATCH("Error activating takeoff sequence")
takingOff = true;
return checkIfCurrentCommandIsCompleted();
}

if(currentCommand.HasExtension(wait_command::wait_command_field))
{
Poco::Timestamp currentTime;
if((currentTime - timeCurrentCommandStarted)/1000000.0 >= currentCommand.GetExtension(wait_command::wait_command_field).wait_duration())
{
return checkIfCurrentCommandIsCompleted();
}

return checkIfCurrentCommandIsCompleted();
}

if(currentCommand.HasExtension(landing_command::landing_command_field))
{
printf("Landing\n");
if(state != LANDED && state != LANDING)
{
printf("Activating landing sequence!!!!!!!!!!!!!\n");
SOM_TRY
activateLandingSequence();
SOM_CATCH("Error activating landing sequence")
landing = true;
}
return checkIfCurrentCommandIsCompleted();
}

return checkIfCurrentCommandIsCompleted();
}


/**
This function takes care of low level behavior that depends on the specific state variables (such as reaching the desired altitude and orientation).

@exceptions: This function can throw exceptions
*/
void soaringPenController::handleLowLevelBehavior()
{
printf("I was called\n");

if(state == LANDED && currentCommand.HasExtension(landing_command::landing_command_field))
{
onTheGroundWithMotorsOff = true;
takingOff = false;
//maintainAltitude = false;
landing = false;
}

maintainAltitude = true;

if(controlEngineIsDisabled || onTheGroundWithMotorsOff || emergencyStopTriggered || shutdownControlEngine)
{
printf("No control %d %d %d\n", controlEngineIsDisabled,  onTheGroundWithMotorsOff, emergencyStopTriggered);
return; //Return if we shouldn't be trying to fly
}

double zThrottle = 0.0;


/*
if(maintainAltitude && (state == FLYING && altitude > 300.0))
{
double pTerm = (targetAltitude - fabs(altitude));
targetAltitudeITerm = targetAltitudeITerm + pTerm;

zThrottle = pTerm/600.0 + targetAltitudeITerm/100000000.0; //PI control for altitude

//printf("Altitude values: target: %.1lf Current: %.1lf Diff: %.1lf throt:  %.1lf I:%.1lf\n", targetAltitude, altitude, targetAltitude -altitude, zThrottle, targetAltitudeITerm); 
}
*/

//Land if tracking lost
if(framesSinceDroneDetected > 15)
{
printf("DRONE LOST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
SOM_TRY
//Add land command
landing_command commandToLand;
command command1(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command1.MutableExtension(landing_command::landing_command_field)) = commandToLand;
command1.set_priority(HIGH_PRIORITY_COMMAND_CONSTANT);

commandQueue.push(command1);

//Add clear command queue command
clear_command_queue_command commandToClearQueue;
command command2(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command2.MutableExtension(clear_command_queue_command::clear_command_queue_command_field)) = commandToClearQueue;
command2.set_priority(HIGH_PRIORITY_COMMAND_CONSTANT);

commandQueue.push(command2);
SOM_CATCH("Error triggering emergency landing and clearing command queue")
return;
}

//Maintain orientation toward desired position
double zRotationThrottle = -targetOrientationInLocalCoordinates.val[0];
//double zRotationThrottle = 0;


//printf("Rotation throttle: %lf\n", zRotationThrottle);

bool followingPath = currentCommand.HasExtension(follow_path_command::follow_path_command_field);


//double xThrottle = xHeading;
//double yThrottle = yHeading;

double xThrottle = 0;
double yThrottle = 0;



if(followingPath)
{
pathFollowingITerm = pathFollowingITerm + pathTargetPointInLocalRelativeImageCoordinates;

printf("Relative position: %lf, %lf\n", pathTargetPointInLocalRelativeImageCoordinates.val[0], pathTargetPointInLocalRelativeImageCoordinates.val[1]);

//Lab camera height is 3.048, should be linear increase of parameters with height due to shrinking of pixel distances


/*
if((heightOfCameraFromFloor/3.048)*pathTargetPointInLocalRelativeImageCoordinates.mag() < .25)
{//Near to the target point
*/
//Positive x is forward
xThrottle = (heightOfCameraFromFloor/3.048)*(1.0*pathTargetPointInLocalRelativeImageCoordinates.val[0]) -.0005*velocityX; //-.00015*velocityX
printf("X Throttle: %lf\n", xThrottle);

//Positive y is right
yThrottle = (heightOfCameraFromFloor/3.048)*(-1.0*pathTargetPointInLocalRelativeImageCoordinates.val[1]) -.0005*velocityY; //+ .00015*velocityY 
printf("Y Throttle: %lf\n", yThrottle);
/*
}
else
{ //Far from target point, so make sure it doesn't start moving too fast
//Positive x is forward
xThrottle = (heightOfCameraFromFloor/3.048)*(4.0*pathTargetPointInLocalRelativeImageCoordinates.val[0]/(pathTargetPointInLocalRelativeImageCoordinates.mag()*4.0)) -.0008*velocityX; //-.00015*velocityX
printf("X Throttle: %lf\n", xThrottle);

//Positive y is right
yThrottle = (heightOfCameraFromFloor/3.048)*(-4.0*pathTargetPointInLocalRelativeImageCoordinates.val[1]/(pathTargetPointInLocalRelativeImageCoordinates.mag()*4.0)) -.0008*velocityY; //+ .00015*velocityY 
printf("Y Throttle: %lf\n", yThrottle);
}
*/

}


printf("XY throttle: %lf %lf\n", xThrottle, yThrottle);


//Tell the drone how it should move TODO: Change back
setVelocityAndRotation(xThrottle, yThrottle, zThrottle, zRotationThrottle);
}

/**
This function takes video frames from the camera device, publishes them and updates the aruco marker based state estimation information for use by the ROS callbacks.  This function is typically called in a seperate thread.
*/
void soaringPenController::videoProcessingAndPublishingFunction()
{
try
{
while(true) 
{
if(shouldExitFlag)
{
return;
}

if(imageSource.grab() != true)
{
fprintf(stderr, "Error, unable to get image from video source\n");
return;
}

//Get image from camera
if(imageSource.retrieve(sourceImage) != true)
{
fprintf(stderr, "Error getting image\n");
return;
}

std::vector<aruco::Marker> detectedMarkers;

//Detect markers
SOM_TRY
markerDetector.detect(sourceImage, detectedMarkers, cameraIntrinsics, markerSizeInMeters);
SOM_CATCH("Error searching for board in image\n")

//Seach for our marker
bool foundDroneMarker = false;
for(int i=0; i<detectedMarkers.size(); i++)
{

//printf("Found: %d\n", detectedMarkers[i].id);

if(detectedMarkers[i].id == droneMarkerIDNumber)
{
//Found drone marker, so update state
std::lock_guard<std::mutex> lock(arucoDataMutex); //Lock mutex
framesSinceDroneDetected = 0;
getCenterPointAndAxis(detectedMarkers[i], cameraIntrinsics, droneCenter, droneXAxis, droneYAxis);
foundDroneMarker = true;

//Draw marker on drone image
aruco::CvDrawingUtils::draw3dAxis(sourceImage, detectedMarkers[i], cameraIntrinsics);

continue;
}

if(detectedMarkers[i].id == landingPadMarkerIDNumber)
{
std::lock_guard<std::mutex> lock(arucoDataMutex); //Lock mutex
landingAreaHasBeenDetected = true;
getCenterPointAndAxis(detectedMarkers[i], cameraIntrinsics, landingAreaCenter, landingAreaXAxis, landingAreaYAxis);
aruco::CvDrawingUtils::draw3dAxis(sourceImage, detectedMarkers[i], cameraIntrinsics);
}


}

if(!foundDroneMarker)
{
std::lock_guard<std::mutex> lock(arucoDataMutex); //Lock mutex
framesSinceDroneDetected++; //Didn't see the marker this frame
}

//Draw path location buffer for debugging
cv::circle(sourceImage, cv::Point((pathLocationBuffer.val[0]*CAMERA_IMAGE_DIAGONAL_SIZE)+CAMERA_IMAGE_WIDTH/2.0, (pathLocationBuffer.val[1]*CAMERA_IMAGE_DIAGONAL_SIZE)+CAMERA_IMAGE_HEIGHT/2.0), 10, cv::Scalar(255,0,0), 5);

//Draw drone location for debugging
cv::circle(sourceImage, cv::Point((droneCenter.val[0]*CAMERA_IMAGE_DIAGONAL_SIZE)+CAMERA_IMAGE_WIDTH/2.0, (droneCenter.val[1]*CAMERA_IMAGE_DIAGONAL_SIZE)+CAMERA_IMAGE_HEIGHT/2.0), 10, cv::Scalar(0,255,0), 5);

//printf("Drawing circle at %lf %lf\n", (pathLocationBuffer.val[0]*CAMERA_IMAGE_DIAGONAL_SIZE)+CAMERA_IMAGE_WIDTH/2.0, (pathLocationBuffer.val[1]*CAMERA_IMAGE_DIAGONAL_SIZE)+CAMERA_IMAGE_HEIGHT/2.0);

//todototodo

//Compress/encode it for transmission
std::vector<unsigned char> encodedImage;
std::vector<int> options; //Pairs of format type:value

//Set jpg quality
options.push_back(CV_IMWRITE_JPEG_QUALITY);
options.push_back(95);

if(cv::imencode(".jpg", sourceImage, encodedImage, options) != true)
{
fprintf(stderr, "Error encoding image\n");
return;
}

//Publish image
SOM_TRY
videoPublisher->send(encodedImage.data(), encodedImage.size());
SOM_CATCH("Error publishing image\n");

}

}
catch(const std::exception &inputException)
{
fprintf(stderr, "%s\n", inputException.what());
return;
}

}


/**
This function repeatedly calls ros::spinOnce until the shouldExitFlag is set (usually by the object destructor).
*/
void soaringPenController::initializeAndRunSpinThread()
{
printf("YAY!!!!!!!!!!!!!!!!!!\n");

bool messageReceived = false;
bool messageDeserialized = false;
while(shouldExitFlag != true)
{
//Check if there is a new command message and add it to the queue if so.
gui_command receivedCommand;
std::tie(messageReceived, messageDeserialized) = receiveProtobufMessage(*commandReceiver,receivedCommand, ZMQ_NOBLOCK);

//TODO: Handle the return to home case

if(receivedCommand.has_follow_path_command_field() )
{ //Add follow path command to queue
if(landingAreaHasBeenDetected && onTheGroundWithMotorsOff)
{ //Ignore path command if landing area hasn't been seen or the drone isn't ready
follow_path_command extendedCommand; //path command after start and finish points have been added
fPoint landingAreaCenterBuffer;
fPoint droneLocationBuffer;

{
std::lock_guard<std::mutex> lock(arucoDataMutex); //Lock mutex
landingAreaCenterBuffer = landingAreaCenter;
droneLocationBuffer = droneCenter;
}

//Add the drone's current location area to the path
extendedCommand.add_path_x_coordinates(droneLocationBuffer.val[0]);
extendedCommand.add_path_y_coordinates(droneLocationBuffer.val[1]);

follow_path_command *originalCommandRef = receivedCommand.mutable_follow_path_command_field();

for(int i=0; i<originalCommandRef->path_x_coordinates_size() && i<originalCommandRef->path_y_coordinates_size(); i++)
{
extendedCommand.add_path_x_coordinates(originalCommandRef->path_x_coordinates(i));
extendedCommand.add_path_y_coordinates(originalCommandRef->path_y_coordinates(i));
}

//Add the landing area to path
extendedCommand.add_path_x_coordinates(landingAreaCenterBuffer.val[0]);
extendedCommand.add_path_y_coordinates(landingAreaCenterBuffer.val[1]);

//Add takeoff command
takeoff_command commandToTakeoff;
command command0(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command0.MutableExtension(takeoff_command::takeoff_command_field)) = commandToTakeoff;

commandQueue.push(command0);

//Add set altitude command to get the quad to keep a certain distance from the ground
/*
set_target_altitude_command altitudeCommand;
altitudeCommand.set_target_altitude(1500.0); //Altitude in mm
command command4(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command4.MutableExtension(set_target_altitude_command::set_target_altitude_command_field)) = altitudeCommand;
commandQueue.push(command4);
*/

//Add a wait command to give a little time for the drone to get in the air
/*
wait_command commandToWait;
commandToWait.set_wait_duration(1);
command command3(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command3.MutableExtension(wait_command::wait_command_field)) = commandToWait;
commandQueue.push(command3);
*/

//Add extended follow path command
command command1(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command1.MutableExtension(follow_path_command::follow_path_command_field)) = extendedCommand;

commandQueue.push(command1);


//Add land command
landing_command commandToLand;
command command2(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command2.MutableExtension(landing_command::landing_command_field)) = commandToLand;

commandQueue.push(command2);
}
}

if(receivedCommand.has_emergency_stop_command_field() )
{ //Add emergency stop command to queue with high priority
command commandToAdd(commandCounter);
commandToAdd.set_priority(HIGH_PRIORITY_COMMAND_CONSTANT);
commandCounter++; //Ensure increasing command numbers
(*commandToAdd.MutableExtension(emergency_stop_command::emergency_stop_command_field)) = *receivedCommand.mutable_emergency_stop_command_field();

commandQueue.push(commandToAdd);
}

if(receivedCommand.has_return_to_home() && receivedCommand.return_to_home() == true)
{
//Time to go home, so add path to home (HIGH_PRIORITY), a landing command (HIGH_PRIORITY), a command to clear all commands after that (HIGH_PRIORITY)
follow_path_command returnHomePathCommand; //path command with current drone position -> landing
fPoint landingAreaCenterBuffer;
fPoint droneLocationBuffer;

{
std::lock_guard<std::mutex> lock(arucoDataMutex); //Lock mutex
landingAreaCenterBuffer = landingAreaCenter;
droneLocationBuffer = droneCenter;
}

//Drone position -> landing position
returnHomePathCommand.add_path_x_coordinates(droneLocationBuffer.val[0]);
returnHomePathCommand.add_path_y_coordinates(droneLocationBuffer.val[1]);

returnHomePathCommand.add_path_x_coordinates(landingAreaCenterBuffer.val[0]);
returnHomePathCommand.add_path_y_coordinates(landingAreaCenterBuffer.val[1]);

command command0(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command0.MutableExtension(follow_path_command::follow_path_command_field)) = returnHomePathCommand;
command0.set_priority(HIGH_PRIORITY_COMMAND_CONSTANT);

commandQueue.push(command0);

//Add land command
landing_command commandToLand;
command command1(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command1.MutableExtension(landing_command::landing_command_field)) = commandToLand;
command1.set_priority(HIGH_PRIORITY_COMMAND_CONSTANT);

commandQueue.push(command1);

//Add clear command queue command
clear_command_queue_command commandToClearQueue;
command command2(commandCounter);
commandCounter++; //Ensure increasing command numbers
(*command2.MutableExtension(clear_command_queue_command::clear_command_queue_command_field)) = commandToClearQueue;
command2.set_priority(HIGH_PRIORITY_COMMAND_CONSTANT);

commandQueue.push(command2);
}

//Process any ros callbacks
ros::spinOnce();
}
}


/**
This point takes a marker and set of camera intrinsics that were used to generate the marker and stores the centerpoint of the marker, as well as its x axis and y axis in the image.
@param inputMarker: The marker to process to establish position
@param inputCameraIntrinsics: The intrinsics of the camera used with marker
@param inputCenterPointBuffer: The variable to store the resulting center point in
@param inputXAxisBuffer: The variable to store the resulting 2d X axis in
@param inputYAxisBuffer: The variable to store the resulting 2d Y axis in
*/
void soaringPen::getCenterPointAndAxis(aruco::Marker &inputMarker, aruco::CameraParameters &inputCameraIntrinsics, soaringPen::fPoint &inputCenterPointBuffer, soaringPen::fPoint &inputXAxisBuffer, soaringPen::fPoint &inputYAxisBuffer)
{
//Mostly adapted from the draw3dAxis function in aruco

float size = inputMarker.ssize * 3;
cv::Mat objectPoints(4, 3, CV_32FC1);
objectPoints.at<float>(0, 0) = 0;
objectPoints.at<float>(0, 1) = 0;
objectPoints.at<float>(0, 2) = 0;
objectPoints.at<float>(1, 0) = size;
objectPoints.at<float>(1, 1) = 0;
objectPoints.at<float>(1, 2) = 0;
objectPoints.at<float>(2, 0) = 0;
objectPoints.at<float>(2, 1) = size;
objectPoints.at<float>(2, 2) = 0;
objectPoints.at<float>(3, 0) = 0;
objectPoints.at<float>(3, 1) = 0;
objectPoints.at<float>(3, 2) = size;

vector<cv::Point2f> imagePoints;
cv::projectPoints(objectPoints, inputMarker.Rvec, inputMarker.Tvec, inputCameraIntrinsics.CameraMatrix, inputCameraIntrinsics.Distorsion, imagePoints);

//Set points
inputCenterPointBuffer.val[0] = (imagePoints[0].x - CAMERA_IMAGE_WIDTH/2.0) / CAMERA_IMAGE_DIAGONAL_SIZE;
inputCenterPointBuffer.val[1] = (imagePoints[0].y - CAMERA_IMAGE_HEIGHT/2.0) / CAMERA_IMAGE_DIAGONAL_SIZE;

inputXAxisBuffer.val[0] = (imagePoints[1].x - imagePoints[0].x) / CAMERA_IMAGE_DIAGONAL_SIZE;
inputXAxisBuffer.val[1] = (imagePoints[1].y - imagePoints[0].y) / CAMERA_IMAGE_DIAGONAL_SIZE;

inputYAxisBuffer.val[0] = (imagePoints[2].x - imagePoints[0].x) / CAMERA_IMAGE_DIAGONAL_SIZE;
inputYAxisBuffer.val[1] = (imagePoints[2].y - imagePoints[0].y) / CAMERA_IMAGE_DIAGONAL_SIZE;
}
