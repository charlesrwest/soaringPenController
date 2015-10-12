#pragma once

#include "fPoint.hpp"
#include<list>
#include "SOMException.hpp"
#include "SOMScopeGuard.hpp"

//For debug
#include<cstdio>

namespace soaringPen
{


/**
This class is intended to represent a linearly interpolated path, defined by a set of points.  It is capable of computing its length as well as its location and x,y derivate along the path.
*/
class linearPath
{
public:

/**
This function adds a point to the path and updates the associated path length.
@param inputPoint: The point to add
*/
void addPoint(const fPoint &inputPoint);

/**
This function removes the given point from the path and updates the associated path length.
@param inputPointIterator: The iterator pointing to the point to remove
*/
void removePoint(const std::list<fPoint>::iterator &inputPointIterator); 

/**
This function takes a point along the path length (ranging from 0.0 to path length) and returns a point between the two points that define the linear arc at that place on the path corresponding to the path length location.
@param inputPathLengthLocation: The location along the path length to find the point for
@return: The associated point.

@throws: This function can throw exceptions
*/
fPoint interpolate(double inputPathLengthLocation);

/**
This function takes a point along the path length (ranging from 0.0 to path length) and returns the X and Y derivative of the path at that location.
@param inputPathLengthLocation: The location along the path length to find the derivative for
@return: The associated derivatives (x,y)

@throws: This function can throw exceptions
*/
fPoint derivative(double inputPathLengthLocation);

/**
This function deletes points from segments that are below the given length (unless there are only two or fewer points left).  This can be used to smooth or linearize a path.
@param inputMinimumSegmentLength: The minimim segment length permitted
*/
void regularize(double inputMinimumSegmentLength);

/**
This function deletes all of the points in the path.
*/
void clear();

/**
This function removes points and interpolates to make the path <= to the given length.
@param inputPathLength: The path length to truncate the path to
*/
void truncate(double inputPathLength);

double pathLength = 0.0;
std::list<fPoint> points;
std::list<double> associatedPathLocations; //Where on the parametric path they fall

private:
/**
This function recalculates the path lengths based on the current points.
*/
void recalculatePathLengths();
};




}
