#include "linearPath.hpp"

using namespace soaringPen;

/**
This function adds a point to the path and updates the associated path length.
@param inputPoint: The point to add
*/
void linearPath::addPoint(const fPoint &inputPoint)
{
points.push_back(inputPoint);
recalculatePathLengths();
}

/**
This function removes the given point from the path and updates the associated path length.
@param inputPointIterator: The iterator pointing to the point to remove
*/
void linearPath::removePoint(const std::list<fPoint>::iterator &inputPointIterator)
{
points.erase(inputPointIterator);
recalculatePathLengths();
}

/**
This function takes a point along the path length (ranging from 0.0 to path length) and returns a point between the two points that define the linear arc at that place on the path corresponding to the path length location.
@param inputPathLengthLocation: The location along the path length to find the point for
@return: The associated point.

@throws: This function can throw exceptions
*/
fPoint linearPath::interpolate(double inputPathLengthLocation)
{
if(inputPathLengthLocation < 0.0)
{
throw SOMException("Negative path length\n", INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

auto pointsIterator = points.begin();
auto associatedPathLocationsIterator = associatedPathLocations.begin();

auto lastPoint = points.begin();
auto lastPointLocation = associatedPathLocations.begin();

while(true)
{
lastPoint = pointsIterator;
lastPointLocation = associatedPathLocationsIterator;
pointsIterator++;
associatedPathLocationsIterator++;

if(pointsIterator == points.end() || associatedPathLocationsIterator == associatedPathLocations.end())
{
break;
}

if(inputPathLengthLocation > *associatedPathLocationsIterator)
{ //The path point further along, so keep going
continue;
}

double ratio = (inputPathLengthLocation - *lastPointLocation)/(*associatedPathLocationsIterator - *lastPointLocation);

return ratio*(*pointsIterator) + (1.0 - ratio)*(*lastPoint);
}


throw SOMException("Path location could not be found\n", INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

/**
This function takes a point along the path length (ranging from 0.0 to path length) and returns the X and Y derivative of the path at that location.
@param inputPathLengthLocation: The location along the path length to find the derivative for
@return: The associated derivatives (x,y)

@throws: This function can throw exceptions
*/
fPoint linearPath::derivative(double inputPathLengthLocation)
{
if(inputPathLengthLocation < 0.0)
{
throw SOMException("Negative path length\n", INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

auto pointsIterator = points.begin();
auto associatedPathLocationsIterator = associatedPathLocations.begin();

auto lastPoint = points.begin();
auto lastPointLocation = associatedPathLocations.begin();

while(true)
{
lastPoint = pointsIterator;
lastPointLocation = associatedPathLocationsIterator;
pointsIterator++;
associatedPathLocationsIterator++;

if(pointsIterator == points.end() || associatedPathLocationsIterator == associatedPathLocations.end())
{
break;
}

if(inputPathLengthLocation > *associatedPathLocationsIterator)
{ //The path point further along, so keep going
continue;
}

fPoint buffer = *pointsIterator - *lastPoint;
buffer.normalize();

return buffer;
}


throw SOMException("Path location could not be found\n", INVALID_FUNCTION_INPUT, __FILE__, __LINE__);
}

/**
This function deletes points from segments that are below the given length (unless there are only two or fewer points left).  This can be used to smooth or linearize a path.
@param inputMinimumSegmentLength: The minimim segment length permitted
*/
void linearPath::regularize(double inputMinimumSegmentLength)
{
if(points.size() <= 2)
{ //There are <= two points, so already done
return;
}

/*
printf("Path segment lengths: ");
for(auto iter = associatedPathLocations.begin(); iter != associatedPathLocations.end(); iter++)
{
printf("%lf ",*iter);
}
printf("\n");
*/

//Ensure path lengths are recalculated
SOMScopeGuard recalculationScopeGuard([&](){recalculatePathLengths();});

for(auto iter = std::next(points.begin(),1); iter != points.end(); iter++)
{
if((*std::prev(iter, 1)).distance(&(*iter)) <= inputMinimumSegmentLength && points.size() > 2)
{ //Delete segment point unless the segment is longer than the limit or there are only two points
auto pointToDelete = iter;
iter = std::prev(iter, 1); //Move iter back to previous point
points.erase(pointToDelete);
}
}

}

/**
This function deletes all of the points in the path.
*/
void linearPath::clear()
{
pathLength = 0.0;
points.clear();
associatedPathLocations.clear();
}

/**
This function removes points and interpolates to make the path <= to the given length.
@param inputPathLength: The path length to truncate the path to
*/
void linearPath::truncate(double inputPathLength)
{
if(pathLength < inputPathLength || inputPathLength < 0.0)
{//Already meets requirements
return;
}

//Path is longer than required, so interpolate a point that meets the requirement, find the first point that is further along than the requirement then remove all points past the requirement and add the interpolated point to the path
fPoint interpolationPoint = interpolate(inputPathLength);

auto locationIter = associatedPathLocations.begin();
for(auto pointIter = points.begin(); pointIter != points.end() && locationIter != associatedPathLocations.end(); (pointIter++, locationIter++))
{
if(*locationIter <= inputPathLength)
{ //Not there yet
continue;
}

//Found first point past, so start deleting
auto pointToDelete = pointIter;
pointIter++; //Move past so the iterator is not affected
points.erase(pointToDelete);
pointIter--;
}

//Add the interpolated point to make the path meet the requirement exactly
addPoint(interpolationPoint);
}

/**
This function recalculates the path lengths based on the current points.
*/
void linearPath::recalculatePathLengths()
{
pathLength = 0.0;
associatedPathLocations.clear();

auto pointsIterator = points.begin();
auto lastPoint = points.begin();

//Add first length of zero for first point
associatedPathLocations.push_back(0.0);

while(true)
{
lastPoint = pointsIterator;
pointsIterator++;

if(pointsIterator == points.end())
{
break;
}

associatedPathLocations.push_back((*lastPoint).distance(&(*pointsIterator)) + *std::prev(associatedPathLocations.end()));
}

pathLength = *std::prev(associatedPathLocations.end());
}
