#include "stdafx.h"
#include "Robot.h"
using namespace std;

#if HAVE_OPENMP || _OPENMP
#include <omp.h>
#endif

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals; 

std::vector<MyObstacle> obsts;


void setupScenario(RVOSimulator* sim)
{
	sim->readSkeleton(skeletonFile);
	/* Specify the global time step of the simulation. */
	sim->setTimeStep(0.2);
	const float ratio_p = 1.0;
	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(10, 20, 10, 0.2, RadiusOfRobot, maxSpeedOfAgent,Vector2(0,0));
	
	IplImage* img = cvLoadImage( fileName.c_str() );
	ifstream fin("C:/Users/rmqlife/Documents/GitHub/skeleton_demo/result/agents.txt");
	int numAgents=0;
	fin>>numAgents;
	for (int i=0; i<numAgents; ++i){
		float startX,startY,goalX,goalY,radius;
		fin>>startY>>startX>>goalY>>goalX>>radius;
		Vector2 start(startX,startY);
		Vector2 goal(goalX,goalY);
		sim->addAgent(start,goal);
		goals.push_back(goal);
	}
	fin.close();
	// copy the im to show
	cv::Mat show(img,true);
	for (size_t i = 0;i < sim->getNumAgents(); i++){
		Vector2 pos = sim->getAgentPosition(i);
		Vector2 goal = sim->getAgentGoal(i);
		cv::circle(show, cv::Point(int(pos.x()),int(pos.y())), int(RadiusOfRobot), cv::Scalar(0,0,255), 2);
		cv::circle(show, cv::Point(int(goal.x()),int(goal.y())), int(RadiusOfRobot), cv::Scalar(255,0,0), 2);
	}
	cv::imshow("show",show);
	cv::waitKey(0);

	sim->setAgentsPathOnSkelton();
	/*deal with obstacle*/
	readObstacle(img);
	for (int i = 0 ; i < obsts.size();i++){
		sim->addObstacle(obsts[i].vtx);
	}
	// Process obstacles so that they are accounted for in the simulation.
	sim->processObstacles();
	srand(time(0));
}

void setPreferredVelocities(RVOSimulator* sim)
{
	/* 
	* Set the preferred velocity to be a vector of unit magnitude (speed) in the
	* direction of the goal.
	*/
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		
		sim->setAgentPrefVelocity(i);
	}
}

bool reachedGoal(RVOSimulator* sim)
{
	/* Check if all agents have reached their goals. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i){
		if ( !sim->checkIfAgentReachGoal(i)){
			return false;
		}
	}
	return true;
}

void readObstacle(IplImage* img){
	int i, j, c;

	
	IplImage* gray = cvCreateImage(cvGetSize(img), 8, 1);
	cvCvtColor(img, gray, CV_BGR2GRAY);

	/// Find contours
	CvSeq* contours = 0;
	CvMemStorage* g_storage = NULL;

	if( g_storage==NULL ) {
		g_storage = cvCreateMemStorage(0);
	} else {
		cvClearMemStorage( g_storage );
	}

	cvThreshold( gray, gray, 100, 255, CV_THRESH_BINARY );

	cvFindContours(gray, g_storage, &contours,sizeof(CvContour),CV_RETR_LIST);
	for (CvSeq *c = contours; c!=NULL; c=c->h_next ){
		MyObstacle obst;
		std::vector<RVO::Vector2> vertices;
		for( int i = 0; i < c->total; ++i ) {
			CvPoint* p = CV_GET_SEQ_ELEM( CvPoint, c, i );
			vertices.push_back(RVO::Vector2(p->x,p->y));
		}
		obst.vtx = vertices;
		obsts.push_back(obst);
		obst.vtx.clear();
		vertices.clear();
	}	
}

