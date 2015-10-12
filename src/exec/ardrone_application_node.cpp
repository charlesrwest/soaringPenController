#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <unistd.h>
#include "soaringPenController.hpp"


//Initialize an object for this to point to before initializing the add command service.
static std::unique_ptr<soaringPen::soaringPenController> mysoaringPenController;


int main(int argc, char** argv)
{

if(argc < 9)
{
fprintf(stderr, "Error, insufficient arguments:\nUsage: %s inputCameraDeviceNumber droneMarkerIDNumber landingAreaMarkerIDNumber pathToCameraCalibration heightOfCameraFromFloorInMeters markerSizeInMeters guiInterfacePortNumber videoPublishingPortNumber\n", argv[0]);
return 1;
}

int videoDeviceNumber = 0;
SOM_TRY
videoDeviceNumber = std::stol(std::string(argv[1]));
SOM_CATCH("Error, unable to read video device number\n")

int droneMarkerIDNumber = 0;
SOM_TRY
droneMarkerIDNumber = std::stol(std::string(argv[2]));
SOM_CATCH("Error, unable to read drone marker id number\n")

int landingAreaMarkerIDNumber = 0;
SOM_TRY
landingAreaMarkerIDNumber = std::stol(std::string(argv[3]));
SOM_CATCH("Error, unable to read landing area marker id number\n")

std::string cameraIntrinsicsFilePath = argv[4];

double heightOfCameraFromFloor = atof(argv[5]);
if(fabs(heightOfCameraFromFloor) < .001)
{
fprintf(stderr, "Error, heightOfCameraFromFloor is invalid\n");
return 1;
} 

double markerSizeInMeters = atof(argv[6]);
if(fabs(markerSizeInMeters) < .001)
{
fprintf(stderr, "Error, marker size is invalid\n");
return 1;
} 



int guiInterfacePortNumber = 0;
SOM_TRY
guiInterfacePortNumber = std::stol(std::string(argv[7]));
SOM_CATCH("Error, unable to read guiInterfacePortNumber\n")

int videoPublishingPortNumber = 0;
SOM_TRY
videoPublishingPortNumber = std::stol(std::string(argv[8]));
SOM_CATCH("Error, unable to read videoPublishingPortNumber\n")

ros::init(argc, argv, "soaring_pen_controller");
ros::NodeHandle nodeHandle;


try
{
printf("Initializing controller node\n");
mysoaringPenController.reset(new soaringPen::soaringPenController(videoDeviceNumber, droneMarkerIDNumber, landingAreaMarkerIDNumber, cameraIntrinsicsFilePath, heightOfCameraFromFloor, markerSizeInMeters, guiInterfacePortNumber, videoPublishingPortNumber));
printf("Initialization completed\n");
}
catch(const std::exception &inputException)
{
fprintf(stderr, "%s\n", inputException.what());
}


while(true)
{

sleep(10); //Sleep while callbacks handle everything
}




ros::shutdown();

return 0;
}





