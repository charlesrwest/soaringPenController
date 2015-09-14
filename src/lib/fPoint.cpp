#include "fPoint.hpp"

using namespace soaringPen;

/**
This function initializes the fPoint to all zeros in the absense of input
*/
fPoint::fPoint()
{
val[0]=0.0;
val[1]=0.0;
}

/**
This function initializes the fPoint using the three scalars provided
@param inputX: This goes to val[0]
@param inputY: This goes to val[1]
*/
fPoint::fPoint(double inputX, double inputY)
{
val[0]=inputX;
val[1]=inputY;
}

/**
This function initializes the fPoint from 3 values in an array
@param inputArray: A pointer to an array of doubles
*/
fPoint::fPoint(double *inputArray)
{
val[0]=inputArray[0];
val[1]=inputArray[1];
}

/**
This function initializes the fPoint from 3 values in an array
@param inputArray: A pointer to an array of integers (converts)
*/
fPoint::fPoint(int *inputArray)
{
val[0]=(double) inputArray[0];
val[1]=(double) inputArray[1];
}

/**
This function normalizes this vector
*/
void fPoint::normalize()
{
double magBuffer=mag();
for(int i=0; i<numberOfElements; i++)
{
val[i]=val[i]/magBuffer;
}
}

/**
This function returns the magnitude of this vector.
@return: The magnitude of this vector.
*/
double fPoint::mag(void) const
{
double sum = 0.0;
for(int i=0; i<numberOfElements; i++)
{
sum = sum + val[i]*val[i];
}
return sqrt(sum);
}

/**
This function returns the angle of the vector relative to the x axis going counterclockwise (valid for 2d).
*/
double fPoint::angle(void) const
{
//A dot B = a1*b1 + a2*b2 ... with x axis a1 =1 a2=0
//Get cos(theta)
double cosTheta=val[0]/mag();
double angle = acos(cosTheta);

//If this vector is in the third or forth quadrant, the angle from acos isn't valid as the absolute angle of this vector and needs to be adjusted.
if(val[1] < 0.0)
{
angle=-angle;
angle=angle+2*PI;
}

return angle;
}

/**
This function returns the distance from this point to the inputted one
@param inputPoint: The point to compute distance from
@return: The distance from that point
*/
double fPoint::distance(fPoint *inputPoint)
{
return sqrt(pow((inputPoint->val[0]-val[0]),2.0)+pow((inputPoint->val[1]-val[1]),2.0));
}

/**
This function returns the distance squared from this point to the inputted one
@param inputPoint: The point to compute distance squared from
@return: The distance squared from that point
*/
double fPoint::distanceS(fPoint *inputPoint)
{
return pow((inputPoint->val[0]-val[0]),2.0)+pow((inputPoint->val[1]-val[1]),2.0);
}


/**
This function allows these vectors to compute the dot product
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the dot product of the two points
*/
double soaringPen::dot(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide)
{
double sum=0.0;
for(int i=0; i<inputLeftHandSide.numberOfElements; i++)
{
sum=sum+inputLeftHandSide.val[i]*inputRightHandSide.val[i];
}

return sum;
}

/**
This function returns the angle between this vector and another, going counter clockwise (in radians)
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: The difference in angle (radians) between the two vectors
*/
double soaringPen::angleDifference(const fPoint &inputLeftHandSide,const fPoint &inputRightHandSide)
{
double angle = inputRightHandSide.angle() - inputLeftHandSide.angle();

//Make sure the angle is positive
if(angle < 0.0)
{
angle=angle+2*PI;
}
return angle;
}


/**
This function allows these vectors to be added.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the sum of the two points
*/
fPoint soaringPen::operator+(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide)
{
return fPoint(inputLeftHandSide.val[0]+inputRightHandSide.val[0],inputLeftHandSide.val[1]+inputRightHandSide.val[1]);
}

/**
This function allows these vectors to be subtracted.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: The difference of the two points
*/
fPoint soaringPen::operator-(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide)
{
return fPoint(inputLeftHandSide.val[0]-inputRightHandSide.val[0],inputLeftHandSide.val[1]-inputRightHandSide.val[1]);
}

/**
This function allows these vectors to be multiplied (element by element).
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the element by element multiplication result
*/
fPoint soaringPen::operator*(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide)
{
return fPoint(inputLeftHandSide.val[0]*inputRightHandSide.val[0],inputLeftHandSide.val[1]*inputRightHandSide.val[1]);
}

/**
This function allows these vectors to be divided (element by element).
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: The element by element quotion
*/
fPoint soaringPen::operator/(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide)
{
return fPoint(inputLeftHandSide.val[0]/inputRightHandSide.val[0],inputLeftHandSide.val[1]/inputRightHandSide.val[1]);
}

/**
This function multiplies this vector by a scalar.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: the vector multiplied by a scalar
*/
fPoint soaringPen::operator*(const fPoint &inputLeftHandSide, double inputRightHandSide)
{
return fPoint(inputLeftHandSide.val[0]*inputRightHandSide,inputLeftHandSide.val[1]*inputRightHandSide);
}

/**
This function divides this vector by a scalar.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: This vector divided by a scalar
*/
fPoint soaringPen::operator/(const fPoint &inputLeftHandSide, double &inputRightHandSide)
{
return fPoint(inputLeftHandSide.val[0]/inputRightHandSide,inputLeftHandSide.val[1]/inputRightHandSide);
}

/**
This function determines if all elements of the vectors are the same.
@param inputLeftHandSide:  The first point
@param inputRightHandSide: The second point
@return: true if the vectors are the same
*/
bool soaringPen::operator==(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide)
{
for(int i=0; i<inputLeftHandSide.numberOfElements; i++)
{
if(inputLeftHandSide.val[i]!=inputRightHandSide.val[i])
{
return false;
}
}
return true; //Made it to the end without an inequality
}

/**
This function compares the left point to the right one to get lexical order (x, then y)
@param inputLeftHandSide: The left point
@param inputRightHandSide: The right point
@return: true the right point is greater than the left, false otherwise
*/
bool soaringPen::operator<(const fPoint &inputLeftHandSide, const fPoint &inputRightHandSide)
{
return inputLeftHandSide.val[0] < inputRightHandSide.val[0] || (inputLeftHandSide.val[0] == inputRightHandSide.val[0] && inputLeftHandSide.val[1] < inputRightHandSide.val[1]);
}


/**
This function prints out the coordinates of the point in simple text format.
@inputOutStream: The stream to output to
@inputFPoint: The point to print out
@return: The stream after this point has been added to it
*/
std::ostream &soaringPen::operator<<(std::ostream &inputOutStream, const fPoint &inputFPoint)
{
inputOutStream << inputFPoint.val[0] << " " << inputFPoint.val[1];
return inputOutStream;
}

/**
This function multiplies the vector by a scalar.
@param inputRightHandSide: The vector to be multiplied by
@param inputLeftHandSide: The scalar to be multiplied
@return: The resulting vector
*/
fPoint  soaringPen::operator*(double inputLeftHandSide, const fPoint &inputRightHandSide)
{
return fPoint(inputRightHandSide.val[0]*inputLeftHandSide, inputRightHandSide.val[1]*inputLeftHandSide);
}

/**
This function multiplies the vector by a scalar.
@param inputRightHandSide: The vector to be multiplied by
@param inputLeftHandSide: The scalar to be multiplied
@return: The resulting vector
*/
fPoint  soaringPen::operator*(double inputLeftHandSide, const fPoint &inputRightHandSide);


/**
This function returns the Z value of a cross product computed as if the given vectors were in 3D space with their current X and Y coordinates
@param inputVector1: The first vector in the cross product
@param inputVector2: The second vector in the cross product
@return: The Z value resulting
*/
double soaringPen::crossZFactor(const fPoint &inputVector1, const fPoint &inputVector2)
{
return inputVector1.val[0]*inputVector2.val[1] - inputVector1.val[1]*inputVector2.val[0];
}

/**
Tests if 3 sequental points make a right turn
@param inputPoint1: The first point
@param inputPoint2: The second point
@param inputPoint3: The third point
@return: True if the points make a right turn (1,2,3 order), false otherwise
*/
bool soaringPen::makesRightTurn(const fPoint &inputPoint1, const fPoint &inputPoint2, const fPoint &inputPoint3)
{
//Take relative vector 1->2 and cross with 2->3.  If resulting Z component is negative, then a right turn has been made
return crossZFactor(inputPoint2-inputPoint1, inputPoint3-inputPoint2) < 0.0;

//Could also use
//http://en.wikipedia.org/wiki/Graham_scan
//return (inputPoint2.val[0]-inputPoint1.val[0])*(inputPoint3.val[1]-inputPoint1.val[1])-(inputPoint2.val[1]-inputPoint1.val[1])*(inputPoint3.val[0]-inputPoint1.val[0]) < 0.0;
}


/**
This function returns if the points are colinear within.  This is determined by comparing the vector from point 1 to point 2 with the vector from point 2 to point 3
@param inputPoint1: The first point to be compared
@param inputPoint2: The second point to be compared
@param inputPoint3: The third point to be compared
@return: true if points are colinear
*/
bool soaringPen::isColinear(fPoint inputPoint1, fPoint inputPoint2, fPoint inputPoint3)
{
fPoint vector1(inputPoint2-inputPoint1);
vector1.normalize();
fPoint vector2(inputPoint3-inputPoint2);
vector2.normalize();

return (vector1 == vector2);
}


/**
This function returns the lexicographic version of < on the x coordinates of the given points.  It returns the lexicographic equivalent of inputLeftPointXCoordinate < inputRightPointXCoordinate.
@param inputLeftPoint: The left point to be compared
@param inputRightPoint: The right point to be compared
@return: True if lexicographic inputLeftPointXCoordinate < inputRightPointXCoordinate
*/
bool soaringPen::lexicographicXComparison(const fPoint &inputLeftPoint, const fPoint &inputRightPoint)
{
if(inputLeftPoint.val[0] == inputRightPoint.val[0]) //If the Xs are the same
{
return inputLeftPoint.val[1] < inputRightPoint.val[1]; //Return the y result
}
else
{
return inputLeftPoint.val[0] < inputRightPoint.val[0];  //Return the X result
}
}

/**
This function returns the lexicographic version of < on the y coordinates of the given points.  It returns the lexicographic equivalent of inputLeftPointYCoordinate < inputRightPointXCoordinate.
@param inputLeftPoint: The left point to be compared
@param inputRightPoint: The right point to be compared
@return: True if lexicographic inputLeftPointYCoordinate < inputRightPointYCoordinate
*/
bool soaringPen::lexicographicYComparison(const fPoint &inputLeftPoint, const fPoint &inputRightPoint)
{
if(inputLeftPoint.val[1] == inputRightPoint.val[1]) //If the Ys are the same
{
return inputLeftPoint.val[0] < inputRightPoint.val[0]; //Return the X result
}
else
{
return inputLeftPoint.val[1] < inputRightPoint.val[1];  //Return the Y result
}
}

/**
This function returns the lexicographic version of < on the x coordinates of the points that the pointers refer to.  It returns the lexicographic equivalent of inputLeftPointXCoordinate < inputRightPointXCoordinate.
@param inputLeftPoint: The left point to be compared
@param inputRightPoint: The right point to be compared
@return: True if lexicographic inputLeftPointXCoordinate < inputRightPointXCoordinate
*/
bool soaringPen::lexicographicXComparisonP(fPoint *inputLeftPoint, fPoint *inputRightPoint)
{
if(inputLeftPoint->val[0] == inputRightPoint->val[0]) //If the Xs are the same
{
return inputLeftPoint->val[1] < inputRightPoint->val[1]; //Return the y result
}
else
{
return inputLeftPoint->val[0] < inputRightPoint->val[0];  //Return the X result
}
}


/**
This function returns the lexicographic version of < on the y coordinates of the points that the pointers refer to.  It returns the lexicographic equivalent of inputLeftPointYCoordinate < inputRightPointXCoordinate.
@param inputLeftPoint: The left point to be compared
@param inputRightPoint: The right point to be compared
@return: True if lexicographic inputLeftPointYCoordinate < inputRightPointYCoordinate
*/
bool soaringPen::lexicographicYComparisonP(fPoint *inputLeftPoint, fPoint *inputRightPoint)
{
if(inputLeftPoint->val[1] == inputRightPoint->val[1]) //If the Ys are the same
{
return inputLeftPoint->val[0] < inputRightPoint->val[0]; //Return the X result
}
else
{
return inputLeftPoint->val[1] < inputRightPoint->val[1];  //Return the Y result
}
}

