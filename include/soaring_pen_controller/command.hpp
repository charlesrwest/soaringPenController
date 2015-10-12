#pragma once

#include "command_message.pb.h"

namespace soaringPen
{

/**
This class represents an command to be executed.  In an attempt to make it easy to extend the command queue processing to addition command types, the details concerning any given set of events are defined as optional protobuf messages inside a this container.  
*/
class command : public command_message //Can contain optional messages
{
public:
/**
This function initializes the command by storing a secondary priority to break priority ties with the default of -1 to indicate an invalid command.
*/
command();

/**
This function initializes the command by storing a secondary priority to break priority ties.
@param inputCommandNumber: The number of the command (should be increasing) 
*/
command(int64_t inputCommandNumber);

/**
This function initializes the command by storing a secondary priority to break priority ties.  The content of the command is set by the given command message.
@param inputCommandNumber: The number of the command (should be increasing)
@param inputCommandMessage: The command message to store
*/
command(int64_t inputCommandNumber, const command_message &inputCommandMessage);


int64_t commandNumber; //Command number
};

/**
This function returns left command priority > right command priority and breaks ties with left.commandNumber < right.commandNumber.
@param inputLeftCommand: The left side of >
@param inputRightCommand: The right side of >
@return: The result of the comparison
*/
bool operator<(const command &inputLeftCommand, const command &inputRightCommand);


}
