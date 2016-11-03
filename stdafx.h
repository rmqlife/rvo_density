// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <vector>
#include <list>
#include <algorithm>
#include <time.h>
#include <iostream>
#include <fstream>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h> 
#include <string>
#include <sstream>
#include <limits>
using namespace std;

/********************************************************/
const string fileName = "MAPS/1.png";
const string skeletonFile = "MAPS/skeleton.txt";
const int num_of_agents = 12;
const float densityThreshold = 0.4;

#define ImageWidth 1237
#define ImageHeight 358
const float RadiusOfRobot = 4;
/********************************************************/
const float PI = 3.14159265;
const float yOffset = -0.01;
const size_t maxPredictFrame = 25;

const float maxSpeedOfAgent = 2.5;
const float adaptedPreferSpeedSlot = 0.1;
//const float equalizationSpeed = 0.3;
//const float equalizationCount = predictFrame;

//#include "GeometryAlgorithm.h"

// TODO: reference additional headers your program requires here