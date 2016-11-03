#pragma once
#include "stdafx.h"
#include "RVO.h"
#include <cv.h>
#include <cvaux.h>
#include <highgui.h> 

struct MyObstacle 
{
	std::vector<RVO::Vector2> vtx;
};
using namespace RVO;
#ifndef M_PI
static const float M_PI = 3.14159265358979323846f;
#endif



void setupScenario(RVOSimulator* sim);

void setPreferredVelocities(RVOSimulator* sim);

bool reachedGoal(RVOSimulator* sim);

void definedObstacles();

void readObstacle(IplImage* img);