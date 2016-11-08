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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

/********************************************************/
const string fileName = "C:/Users/rmqlife/Documents/GitHub/skeleton_demo/data/liang2.png";
const string skeletonFile = "C:/Users/rmqlife/Documents/GitHub/skeleton_demo/result/skeleton.txt";
const string agentFile = "C:/Users/rmqlife/Documents/GitHub/skeleton_demo/result/agents.txt";
const float densityThreshold = 0.8;
#define ImageWidth 1237
#define ImageHeight 358
const float RadiusOfRobot = 5;

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
