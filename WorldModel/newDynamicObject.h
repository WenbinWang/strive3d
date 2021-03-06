#ifndef NEWDYNAMICOBCJET_H
#define NEWDYNAMICOBCJET_H
#include "object.h"
#include "types.h"
#include <iostream>
using namespace std;

class NDynamicObject:public Object
{
public:
  NDynamicObject();
  ~NDynamicObject();
  salt::Vector3f pos;
  salt::Vector3f h_pos;
  double vel;
  
public:
  void setGlobalPos(salt::Vector3f g_pos){
    pos = g_pos;}
  void calVel();

  void updateFromVision(VisionSense vs);
  void printVisionModel();
};
#endif