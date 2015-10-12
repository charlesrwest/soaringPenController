#include "command.hpp"

using namespace soaringPen; 

/**
This function initializes the command by storing a secondary priority to break priority ties with the default of -1 to indicate an invalid command.
*/
command::command()
{
commandNumber = -1;
}

/**
This function initializes the command by storing a secondary priority to break priority ties.
@param inputCommandNumber: The number of the command (should be increasing) 
*/
command::command(int64_t inputCommandNumber)
{
commandNumber = inputCommandNumber;
}

/**
This function initializes the command by storing a secondary priority to break priority ties.  The content of the command is set by the given command message.
@param inputCommandNumber: The number of the command (should be increasing)
@param inputCommandMessage: The command message to store
*/
command::command(int64_t inputCommandNumber, const command_message &inputCommandMessage) : command_message(inputCommandMessage)
{
commandNumber = inputCommandNumber;
}


/**
This function returns left command priority > right command priority and breaks ties with left.commandNumber < right.commandNumber.
@param inputLeftCommand: The left side of >
@param inputRightCommand: The right side of >
@return: The result of the comparison
*/
bool soaringPen::operator<(const command &inputLeftCommand, const command &inputRightCommand)
{
if(inputLeftCommand.priority() == inputRightCommand.priority())
{ //Lower is better
return inputLeftCommand.commandNumber > inputRightCommand.commandNumber;
}

//Higher is better
return inputLeftCommand.priority() < inputRightCommand.priority();
}

