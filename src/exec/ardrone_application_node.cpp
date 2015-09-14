#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <unistd.h>
#include "soaringPenController.hpp"
#include "ardrone_command/serialized_ardrone_command.h"
#include "ardrone_command/commandInterface.h"


//Initialize an object for this to point to before initializing the add command service.
static std::unique_ptr<soaringPen::soaringPenController> mysoaringPenController;


int main(int argc, char** argv)
{


ros::init(argc, argv, "soaring_pen_controller");
ros::NodeHandle nodeHandle;


try
{
printf("Initializing controller node\n");
mysoaringPenController.reset(new soaringPen::soaringPenController(1, 178, 9001, 9002));
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





