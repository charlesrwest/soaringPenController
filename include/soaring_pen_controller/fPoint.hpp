#pragma once

#include<cmath>
#define PI 3.14159265
#include<cstdio>
#include<iostream>
#include<stdlib.h>
#include<limits.h>

namespace soaringPen
{


/**
This class is my own version of the generic 2 dimensional floating point vector (recycled from vision mosaics machine learning then PRT simulation)
*/
class fPoint
{
public:
const static int numberOfElements = 2;

/**
This function initializes the fPoint to all zeros in the absense of input
*/
fPoint();

/**
This function initializes the fPoint using the three scalars provided
@param inputX: This goes to val[0]
@param inputY: This goes to val[1]
*/
fPoint(double inputX, double inputY);

/**
This function initializes the fPoint from 2 values in an array
@param inputArray: A pointer to an array of doubles
*/
fPoint(double *inputArray);

/**
This function initializes the fPoint from 2 values in an array
@param inputArray: A pointer to an array of integers (converts)
*/
fPoint(int *inputArray);

/**
This function normalizes this vector
*/
void normalize();

/**
This function returns the magnitude of this vector.
@return: The magnitude of this vector.
*/
double mag(void) const;

/**
This function returns the angle of the vector relative to the x axis going counterclockwise (valid for 2d).
*/
double angle(void) const;

/**
This function returns the distance from this point to the inputted one
@param inputPoint: The point to compute distance from
@return: The distance from that point
*/
double distance(fPoint *inputPoint);

/**
This function returns the distance squared from this point to the inputted one
@param inputPoint: The point to compute distance squared from
@return: The distance squared from that point
*/
double distanceS(fPoint *inputPoint);

/**
This function allows these vectors to compute the dot product
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the dot product of the two points
*/
friend double dot(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function returns the angle between this vector and another, going counter clockwise (in radians)
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: The difference in angle (radians) between the two vectors
*/
friend double angleDifference(const fPoint &inputLeftHandSide,const fPoint &inputRightHandSide);

/**
This function allows these vectors to be added.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the sum of the two points
*/
friend fPoint operator+(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function allows these vectors to be subtracted.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: The difference of the two points
*/
friend fPoint operator-(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function allows these vectors to be multiplied (element by element).
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the element by element multiplication result
*/
friend fPoint operator*(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function allows these vectors to be divided (element by element).
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: The element by element quotion
*/
friend fPoint operator/(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function multiplies this vector by a scalar.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the vector multiplied by a scalar
*/
friend fPoint operator*(const fPoint &inputLeftHandSide, double inputRightHandSide);

/**
This function divides this vector by a scalar.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: This vector divided by a scalar
*/
friend fPoint operator/(const fPoint &inputLeftHandSide, double &inputRightHandSide);

/**
This function determines if all elements of the vectors are the same.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: true if the vectors are the same
*/
friend bool operator==(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);



/**
This function prints out the coordinates of the point in simple text format.
@inputOutStream: The stream to output to
@inputFPoint: The point to print out
@return: The stream after this point has been added to it
*/
friend std::ostream &operator<<(std::ostream &inputOutStream, const fPoint &inputFPoint);

/**
This function compares the left point to the right one to get lexical order (x, then y)
@param inputLeftHandSide: The left point
@param inputRightHandSide: The right point
@return: true the right point is greater than the left, false otherwise
*/
friend bool operator<(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide); 

double val[2];
};

/**
This function allows these vectors to compute the dot product
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the dot product of the two points
*/
double dot(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function returns the angle between this vector and another, going counter clockwise (in radians)
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: The difference in angle (radians) between the two vectors
*/
double angleDifference(const fPoint &inputLeftHandSide,const fPoint &inputRightHandSide);

/**
This function allows these vectors to be added.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the sum of the two points
*/
fPoint operator+(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function allows these vectors to be subtracted.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: The difference of the two points
*/
fPoint operator-(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function allows these vectors to be multiplied (element by element).
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the element by element multiplication result
*/
fPoint operator*(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function multiplies the vector by a scalar.
@param inputRightHandSide: The vector to be multiplied by
@param inputLeftHandSide: The scalar to be multiplied
@return: The resulting vector
*/
fPoint  operator*(double inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function allows these vectors to be divided (element by element).
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: The element by element quotion
*/
fPoint operator/(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);

/**
This function multiplies this vector by a scalar.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the vector multiplied by a scalar
*/
fPoint operator*(const fPoint &inputLeftHandSide, double inputRightHandSide);

/**
This function divides this vector by a scalar.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: This vector divided by a scalar
*/
fPoint operator/(const fPoint &inputLeftHandSide, double &inputRightHandSide);

/**
This function determines if all elements of the vectors are the same.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: true if the vectors are the same
*/
bool operator==(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide);



/**
This function prints out the coordinates of the point in simple text format.
@inputOutStream: The stream to output to
@inputFPoint: The point to print out
@return: The stream after this point has been added to it
*/
std::ostream &operator<<(std::ostream &inputOutStream, const fPoint &inputFPoint);

/**
This function compares the left point to the right one to get lexical order (x, then y)
@param inputLeftHandSide: The left point
@param inputRightHandSide: The right point
@return: true the right point is greater than the left, false otherwise
*/
bool operator<(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide); 


/**
This function returns the Z value of a cross product computed as if the given vectors were in 3D space with their current X and Y coordinates
@param inputVector1: The first vector in the cross product
@param inputVector2: The second vector in the cross product
@return: The Z value resulting
*/
double crossZFactor(const fPoint &inputVector1, const fPoint &inputVector2); 

/**
Tests if 3 sequental points make a right turn
@param inputPoint1: The first point
@param inputPoint2: The second point
@param inputPoint3: The third point
@return: True if the points make a right turn (1,2,3 order), false otherwise
*/
bool makesRightTurn(const fPoint &inputPoint1, const fPoint &inputPoint2, const fPoint &inputPoint3);

/**
This function returns if the points are colinear within.  This is determined by comparing the vector from point 1 to point 2 with the vector from point 2 to point 3
@param inputPoint1: The first point to be compared
@param inputPoint2: The second point to be compared
@param inputPoint3: The third point to be compared
@return: true if points are colinear
*/
bool isColinear(fPoint inputPoint1, fPoint inputPoint2, fPoint inputPoint3);


/**
This function returns the lexicographic version of < on the x coordinates of the given points.  It returns the lexicographic equivalent of inputLeftPointXCoordinate < inputRightPointXCoordinate.
@param inputLeftPoint: The left point to be compared
@param inputRightPoint: The right point to be compared
@return: True if lexicographic inputLeftPointXCoordinate < inputRightPointXCoordinate
*/
bool lexicographicXComparison(const fPoint &inputLeftPoint, const fPoint &inputRightPoint);

/**
This function returns the lexicographic version of < on the y coordinates of the given points.  It returns the lexicographic equivalent of inputLeftPointYCoordinate < inputRightPointXCoordinate.
@param inputLeftPoint: The left point to be compared
@param inputRightPoint: The right point to be compared
@return: True if lexicographic inputLeftPointYCoordinate < inputRightPointYCoordinate
*/
bool lexicographicYComparison(const fPoint &inputLeftPoint, const fPoint &inputRightPoint);

/**
This function returns the lexicographic version of < on the x coordinates of the points that the pointers refer to.  It returns the lexicographic equivalent of inputLeftPointXCoordinate < inputRightPointXCoordinate.
@param inputLeftPoint: The left point to be compared
@param inputRightPoint: The right point to be compared
@return: True if lexicographic inputLeftPointXCoordinate < inputRightPointXCoordinate
*/
bool lexicographicXComparisonP(fPoint *inputLeftPoint, fPoint *inputRightPoint);

/**
This function returns the lexicographic version of < on the y coordinates of the points that the pointers refer to.  It returns the lexicographic equivalent of inputLeftPointYCoordinate < inputRightPointXCoordinate.
@param inputLeftPoint: The left point to be compared
@param inputRightPoint: The right point to be compared
@return: True if lexicographic inputLeftPointYCoordinate < inputRightPointYCoordinate
*/
bool lexicographicYComparisonP(fPoint *inputLeftPoint, fPoint *inputRightPoint);

}

