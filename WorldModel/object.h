#ifndef OBJECT_WM_H
#define OBJECT_WM_H
#include "vector.h"
using namespace salt;
class Object
{
public:
  	salt::Vector3f pos;
	salt::Vector3f vel;
	salt::Vector3f pos_local;
	double distanceToSelf;
	void SetDistanceToSelf(double distance){distanceToSelf = distance;}
	double GetDistanceToSelf(){return distanceToSelf;}

};
#endif