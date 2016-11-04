/*
 * RVOSimulator.cpp
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */
#include "StdAfx.h"
#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

namespace RVO {
	RVOSimulator::RVOSimulator() : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(0.0f)
	{
		kdTree_ = new KdTree(this);
		predictFrame_ = maxPredictFrame;//(maxSpeedOfAgent / RadiusOfRobot) + 1;
	}

	RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity) : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(timeStep)
	{
		kdTree_ = new KdTree(this);
		defaultAgent_ = new Agent(this);

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;

		predictFrame_ = maxPredictFrame;//(maxSpeedOfAgent / RadiusOfRobot) + 1;
	}

	RVOSimulator::~RVOSimulator()
	{
		if (defaultAgent_ != NULL) {
			delete defaultAgent_;
		}

		for (size_t i = 0; i < agents_.size(); ++i) {
			delete agents_[i];
		}

		for (size_t i = 0; i < obstacles_.size(); ++i) {
			delete obstacles_[i];
		}

		delete kdTree_;
	}

	size_t RVOSimulator::addAgent(const Vector2 &position, const Vector2 &goal)
	{
		if (defaultAgent_ == NULL) {
			//return RVO_ERROR;
		}

		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->adaptedCount = 0;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizon_ = defaultAgent_->timeHorizon_;
		agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
		agent->velocity_ = defaultAgent_->velocity_;

		agent->id_ = agents_.size();
		agent->goal = goal;
		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
	{
		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->timeHorizonObst_ = timeHorizonObst;
		agent->velocity_ = velocity;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	size_t RVOSimulator::addObstacle(const std::vector<Vector2> &vertices)
	{
		if (vertices.size() < 2) {
			//return RVO_ERROR;
		}

		const size_t obstacleNo = obstacles_.size();

		for (size_t i = 0; i < vertices.size(); ++i) {
			Obstacle *obstacle = new Obstacle();
			obstacle->point_ = vertices[i];

			if (i != 0) {
				obstacle->prevObstacle_ = obstacles_.back();
				obstacle->prevObstacle_->nextObstacle_ = obstacle;
			}

			if (i == vertices.size() - 1) {
				obstacle->nextObstacle_ = obstacles_[obstacleNo];
				obstacle->nextObstacle_->prevObstacle_ = obstacle;
			}

			obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);

			if (vertices.size() == 2) {
				obstacle->isConvex_ = true;
			}
			else {
				obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
			}

			obstacle->id_ = obstacles_.size();

			obstacles_.push_back(obstacle);
		}

		return obstacleNo;
	}

	void RVOSimulator::doStep()
	{	
		if (predictFrame_ < maxPredictFrame){//incrementally increase the predicting distance
			predictFrame_++;
		}
		kdTree_->buildAgentTree();

		desityEqualization();
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->setAdaptedPreferVeloicty();//set the adapted velocity
			bool reached = checkIfAgentReachGoal(i);
			if (true == reached){
				continue;
			}
			
			agents_[i]->computeNeighbors();
			agents_[i]->computeNewVelocity();
		}

#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->update();
		}
		
		globalTime_ += timeStep_;
	}

	size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
	}

	size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}

	float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}

	float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}

	size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->agentNeighbors_.size();
	}

	size_t RVOSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_.size();
	}

	size_t RVOSimulator::getAgentNumORCALines(size_t agentNo) const
	{
		return agents_[agentNo]->orcaLines_.size();
	}

	size_t RVOSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
	}

	const Line &RVOSimulator::getAgentORCALine(size_t agentNo, size_t lineNo) const
	{
		return agents_[agentNo]->orcaLines_[lineNo];
	}

	const Vector2 &RVOSimulator::getAgentPosition(size_t agentNo) const
	{
		return agents_[agentNo]->position_;
	}

	const Vector2 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity_;
	}

	float RVOSimulator::getAgentRadius(size_t agentNo) const
	{
		return agents_[agentNo]->radius_;
	}

	float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizon_;
	}

	float RVOSimulator::getAgentTimeHorizonObst(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizonObst_;
	}

	const Vector2 &RVOSimulator::getAgentVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}

	float RVOSimulator::getGlobalTime() const
	{
		return globalTime_;
	}

	size_t RVOSimulator::getNumAgents() const
	{
		return agents_.size();
	}

	size_t RVOSimulator::getNumObstacleVertices() const
	{
		return obstacles_.size();
	}

	const Vector2 &RVOSimulator::getObstacleVertex(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->point_;
	}

	size_t RVOSimulator::getNextObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->nextObstacle_->id_;
	}

	size_t RVOSimulator::getPrevObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->prevObstacle_->id_;
	}

	float RVOSimulator::getTimeStep() const
	{
		return timeStep_;
	}
	bool RVOSimulator::checkIfAgentReachGoal(size_t agentID){
		bool reach = false; 
		int goalID = agents_[agentID]->nextGoalID;
		Vector2 nextGoalPos = agents_[agentID]->subGoals[goalID];
		float ran = 1;
		
		if (goalID != agents_[agentID]->subGoals.size() - 1){
			ran = verticeOnSkelton_[agentsSkeletonPathIDs_[agentID][goalID]].ran;
			if(RVO::abs(this->agents_[agentID]->position_ - nextGoalPos) 
				< ran/4){
				reach = true;
			}
		}
		else{
			if(RVO::abs(this->agents_[agentID]->position_ - nextGoalPos) <= ran){
				reach = true;
			}
		}
		if (true == reach){//if reach a goal
			int nextID = this->agents_[agentID]->getNextGoalID();
			if (-1 == nextID){//already last one 
				return true;//have reached all
			}
			else{
				return false;
			}
		} 
		else{
			return false;//not reach the goal yet
		}
		return reach;
	}
	Vector2 RVOSimulator::getAgentGoal(size_t agentID){
		return agents_[agentID]->getGoal();
	}
	void RVOSimulator::processObstacles()
	{
		kdTree_->buildObstacleTree();
	}

	bool RVOSimulator::queryVisibility(const Vector2 &point1, const Vector2 &point2, float radius) const
	{
		return kdTree_->queryVisibility(point1, point2, radius);
	}

	void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
	{
		if (defaultAgent_ == NULL) {
			defaultAgent_ = new Agent(this);
		}

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;
	}

	void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		agents_[agentNo]->maxNeighbors_ = maxNeighbors;
	}

	void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		agents_[agentNo]->maxSpeed_ = maxSpeed;
	}

	void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		agents_[agentNo]->neighborDist_ = neighborDist;
	}

	void RVOSimulator::setAgentPosition(size_t agentNo, const Vector2 &position)
	{
		agents_[agentNo]->position_ = position;
	}

	void RVOSimulator::setAgentPrefVelocity(size_t agentNo)
	{
		Vector2 nextGoalPos = agents_[agentNo]->subGoals[agents_[agentNo]->nextGoalID];
		RVO::Vector2 prefVector = nextGoalPos - getAgentPosition(agentNo);
		prefVector = normalize(prefVector);
		agents_[agentNo]->prefVelocity_ = prefVector * maxSpeedOfAgent;
		
	}

	void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_ = radius;
	}

	void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
	{
		agents_[agentNo]->timeHorizon_ = timeHorizon;
	}

	void RVOSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
	{
		agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
	}

	void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector2 &velocity)
	{
		agents_[agentNo]->velocity_ = velocity;
	}

	void RVOSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}
	void RVOSimulator::setAgentsPathOnSkelton(){
		size_t nAgents = getNumAgents();	
		IplImage* img = cvLoadImage( fileName.c_str() );
		for (size_t i = 0;i < skeleton_.size();i++){
			
			for (size_t j = 0;j < skeleton_[i].size();j++){
				if (skeleton_[i][j].position.y() < img->height && skeleton_[i][j].position.x() < img->width ){
					cvSet2D(img, skeleton_[i][j].position.y(), skeleton_[i][j].position.x(), cvScalar(0,0,255));
				}
			}
		}
	
		/*search the path*/
		findAllShortestPaths();
		
		
		for (size_t i = 0;i < nAgents;i++){
			/*find the closest points*/
			Vector2 startPos = agents_[i]->position_;
			Vector2 goalPos = agents_[i]->getGoal();
		
			cvCircle(img,cvPoint(startPos.x(),startPos.y()),4,cvScalar(255,0,0));
			cvCircle(img,cvPoint(goalPos.x(),goalPos.y()),4,cvScalar(255,0,0));
			
			int startID;
			int goalID;
			float minPtToStart = INT_MAX;
			float minPtToGoal = INT_MAX;
			/*find the closet nodes*/
			for (size_t j = 0;j < skeleton_.size();j++){
				for (size_t k = 0;k < skeleton_[j].size();k++){
					if (abs(skeleton_[j][k].position - startPos) < minPtToStart){
						startID = j;
						minPtToStart = abs(skeleton_[j][k].position - startPos);
					}
					if (abs(skeleton_[j][k].position - goalPos) < minPtToGoal){
						goalID = j;
						minPtToGoal = abs(skeleton_[j][k].position - goalPos);
					}
				}
			}
			if (startID == goalID){ // start and goal positions are on the same edge
				vector<Vector2> goals;
				goals.push_back(agents_[i]->getGoal());
				agents_[i]->setSubGoals(goals);
				vector<size_t> pathIDs;
				Vector2 goalPose = (skeleton_[startID].end()-1)->position;
				vector<skeletonPoint>::iterator is = 
					find(verticeOnSkelton_.begin(),verticeOnSkelton_.end(),goalPose);
				if (verticeOnSkelton_.end() == is){
					int ttt = 0;
				}
				size_t vertexID = is - verticeOnSkelton_.begin();
				pathIDs.push_back(vertexID);
				agentsSkeletonPathIDs_.push_back(pathIDs);
				vector<Vector2> mappingTrajectory;
				for (size_t v = 0;v < skeleton_[startID].size();v++){
					mappingTrajectory.push_back(skeleton_[startID][v].position);
				}
				agents_[i]->buildTree(mappingTrajectory);
				continue;
			}
			
			Vector2 v00 = (skeleton_[startID].begin())->position;
			Vector2 v01 = (skeleton_[startID].end()-1)->position;

			Vector2 v10 = (skeleton_[goalID].begin())->position;
			Vector2 v11 = (skeleton_[goalID].end()-1)->position;
			/*for (size_t t = 0;t < skeleton[goalID].size();t++){
			float x = skeleton[goalID][t].position.x();
			float y = skeleton[goalID][t].position.y();
			if (x < img->width && y < img->height){
			cvSet2D(img,skeleton[goalID][t].position.y(),skeleton[goalID][t].position.x(),cvScalar(255,255,0));
			}
			}
			cvCircle(img,cvPoint(v10.x(),v10.y()),5,cvScalar(0,0,0));
			cvCircle(img,cvPoint(v11.x(),v11.y()),5,cvScalar(0,0,0));
			cvShowImage("",img);
			cvWaitKey(NULL);*/
			vector<float> d;
			d.push_back(abs(v00 - v10));
			d.push_back(abs(v00 - v11));
			d.push_back(abs(v01 - v10));
			d.push_back(abs(v01 - v11));
			size_t maxID = -1;
			float maxDst = -1*INT_MAX;
			for (size_t j = 0;j < d.size();j++){
				if (d[j] > maxDst){
					maxDst = d[j];
					maxID = j;
				}
			}
			if (maxID == 0){
				startPos = v00;
				goalPos = v10;
			}
			if (maxID == 1){ 
				startPos = v00;
				goalPos = v11;
			}
			if (maxID == 2){
				startPos = v01;
				goalPos = v10;
			}
			if (maxID == 3){
				startPos = v01;
				goalPos = v11;
			}
			for (size_t j = 0;j < verticeOnSkelton_.size();j++){
				if (verticeOnSkelton_[j].position == startPos){
					startID = j;
				}
				if (verticeOnSkelton_[j].position == goalPos){
					goalID = j;
				}
			}
			/*find the path*/
			vector<size_t> pathIDs;
			vector<size_t> shortestTreeOfGoalID = shortestPaths_[goalID];
			size_t curID = startID;
			pathIDs.push_back(curID);
			while (1){
				curID = shortestTreeOfGoalID[curID];
				pathIDs.push_back(curID);
				if(curID == goalID){
					break;
				}
			}
			vector<size_t> edgeIDs;
			for (size_t j = 0;j < pathIDs.size()-1;j++){
				Vector2 v0 = verticeOnSkelton_[pathIDs[j]].position;
				Vector2 v1 = verticeOnSkelton_[pathIDs[j+1]].position;
				
				for (size_t k = 0;k < skeleton_.size();k++){
					if (skeleton_[k].begin()->position == v0 && (skeleton_[k].end()-1)->position == v1){
						edgeIDs.push_back(k);
						/*draw it*/
						/*for (size_t m = 0;m < skeleton[k].size();m++){
						cvSet2D(img,skeleton[k][m].position.y(),skeleton[k][m].position.x(),cvScalar(0,0,0));
						}
						cvShowImage("",img);
						cvWaitKey(NULL);*/
						break;
					}
				}
			}
			vector<Vector2> mappingTrajectory;
			for (size_t j = 0;j < edgeIDs.size();j++){
				for (size_t k = 0;k < skeleton_[edgeIDs[j]].size();k++){
					mappingTrajectory.push_back(Vector2(skeleton_[edgeIDs[j]][k].position));
				}
			}
			
			agents_[i]->buildTree(mappingTrajectory);
			//(agentsPathTree.end()-1)->buildTree(mappingTrajectory);
			//pathTree.buildTree(mappingTrajectory);
			//agentsPathTree.push_back(pathTree);//push the edges' ID that agents will follow during the moving 
			vector<Vector2> goals;
			pathIDs.erase(pathIDs.begin());
			pathIDs.pop_back();
			for (size_t j = 0;j < pathIDs.size();j++){
				goals.push_back(verticeOnSkelton_[pathIDs[j]].position);
			}
			goals.push_back(agents_[i]->getGoal());
			agents_[i]->setSubGoals(goals);
			agentsSkeletonPathIDs_.push_back(pathIDs);
			
		}
	}
	void RVOSimulator::readSkeleton(string sltfile){
		ifstream ifile(sltfile.c_str(),ios::in);
		size_t nPts = -1;
		ifile>>nPts;
		

		IplImage* img = cvLoadImage( fileName.c_str() );
		const int dim = 2;
		
		vector<Vector2> pts;
		for (size_t c = 0;c < nPts;c++){
			skeletonPoint pt;
			ifile>>pt.position.x_>>pt.position.y_>>pt.ran;
			sltPts_.push_back(pt);
			if (pt.position.x()<img->width && pt.position.y()<img->height){
				cvSet2D(img, pt.position.y(), pt.position.x(), cvScalar(0,0,0));
			}
			pts.push_back(Vector2(pt.position.x_, pt.position.y_));
		}
		skeletonKDTree_.buildTree(pts);
	
		/*find the vertice*/
		vector<size_t> verticeIDs;
		for (size_t i = 0;i < nPts;i++){
			for (size_t j = 0;j < nPts;j++){
				if (j == i){
					continue;
				}
				if (abs(sltPts_[i].position - sltPts_[j].position) <= sqrt(2.0)){
					sltPts_[i].connectedIDs.push_back(j);
				}
			}
			if (sltPts_[i].connectedIDs.size() != 2){//find a vertex
				verticeIDs.push_back(i);
				cvCircle(img, cvPoint(sltPts_[i].position.x(), sltPts_[i].position.y()), 5,cvScalar(255,0,0),2);
			}
		}

		/*find the connections*/
		vector<bool> visitedVertice(nPts,false);
		
		size_t nVisited = 1;
		size_t visitingVertexID = verticeIDs[0];
		visitedVertice[visitingVertexID] = true;
		for (size_t i = 0;i < verticeIDs.size();i++){//from a vertex of skeleton
			skeletonPoint visitingVertex = sltPts_[verticeIDs[i]];
			vector<bool> visitedPts(nPts,false);
			visitedPts[verticeIDs[i]] = true;
			for (size_t j = 0;j < visitingVertex.connectedIDs.size();j++){// search around the neighboring points of vertex
				vector<skeletonPoint> edge;
				skeletonPoint visitingPt = sltPts_[verticeIDs[i]];
				edge.push_back(visitingPt);
				while (1){
					for (size_t k = 0;k < visitingPt.connectedIDs.size();k++){
						size_t ID = visitingPt.connectedIDs[k];
						if (visitedPts[ID] == false){
							visitingPt = sltPts_[ID];
							visitedPts[ID] = true;
							edge.push_back(sltPts_[ID]);
							break;
						}
					}
					
					if (visitingPt.connectedIDs.size() != 2){
						break;
					} 
				}
				
				skeleton_.push_back(edge);
				vector<skeletonPoint> t_edge = edge;
				edge.clear();
				for (int k = t_edge.size()-1;k >= 0;k-- ){
					edge.push_back(t_edge[k]);
				}
				skeleton_.push_back(edge);
				/*for (size_t i = 0;i < edge.size();i++){
					Vector2 pt = edge[i].position;
					if (pt.y() < img->height && pt.x() < img->width ){
						cvSet2D(img, pt.y(),pt.x(), cvScalar(0,255,255));
					}
				}
				cvShowImage("",img);
				cvWaitKey(NULL);*/
			}
		}
		
	}
	void RVOSimulator::findAllShortestPaths(){
		vector< vector<int> > initialDst;
		vector<skeletonPoint> visitedPoses;
		for (size_t i = 0;i < skeleton_.size();i++){
		//	cout<<skeleton_[i][0].position.x()<<" "<<skeleton_[i][0].position.y()<<"    "
		//		<<(skeleton_[i].end()-1)->position.x()<<" "<<(skeleton_[i].end()-1)->position.y()<<endl;
			if (visitedPoses.end() != find(visitedPoses.begin(),
				visitedPoses.end(),skeleton_[i][0])){
					continue;
			}
			else{
				visitedPoses.push_back(skeleton_[i][0]);//mark it
			}
		}
		for (size_t i = 0;i < visitedPoses.size();i++){
			Vector2 visitingPos = visitedPoses[i].position;
			vector<int> dst(visitedPoses.size(), -1);
			dst[i] = 0;
			for (size_t j = 0;j < skeleton_.size();j++){
				Vector2 startPos = skeleton_[j][0].position;
				Vector2 goalPos = (skeleton_[j].end()-1)->position;
				if (startPos == visitingPos){
					for (size_t k = 0;k < visitedPoses.size();k++){
						if (visitedPoses[k].position == goalPos){
							dst[k] = skeleton_[j].size();
							break;
						}
					}
				}
			}
		/*	for (size_t j = 0;j < dst.size();j++){
				cout<<dst[j]<<" ";
			}
			cout<<endl;*/
			initialDst.push_back(dst);
		}
		//for all nodes find the shortest tree
		for (size_t i = 0;i < visitedPoses.size();i++){
			vector<size_t>  path = dijkstra(initialDst, i);
			shortestPaths_.push_back(path);
		}
		verticeOnSkelton_ = visitedPoses;
	}
	vector<Vector2> RVOSimulator::calculateMappingPositionOnSkeleton(){
		vector<Vector2> predictedPoses;
		for (size_t i = 0;i < agents_.size();i++){
			Vector2 agentPos = agents_[i]->position_;
			/*prediction*/
			Vector2 predictedPos = agentPos + agents_[i]->prefVelocity_ * predictFrame_;
			float dst = 0;
			Vector2 clostestPos = agents_[i]->searchNearest(predictedPos,dst);
			
			vector<skeletonPoint>::iterator ipttt = find(sltPts_.begin(),sltPts_.end(),clostestPos);
			if (ipttt->ran == 0){
				IplImage* img = cvLoadImage( fileName.c_str() );
				cvCircle(img, cvPoint(ipttt->position.x(),ipttt->position.y()),10, cvScalar(0,0,0),2);
				cvShowImage("",img);
				cvWaitKey(NULL);
				float ttt = 0;
			}
				
			predictedPoses.push_back(clostestPos);
		}
		
		predictedPoses_ = predictedPoses;
		return predictedPoses;
	}
	vector<size_t> RVOSimulator::clusterMappingPositionsOfAgents(vector<float> &density, vector<Vector2>  predictedPoses
		, vector< vector<size_t> > &clusteredAgentsIDByMappingPoses){
		kdTreeGeneral predictedPosesTree; 
		predictedPosesTree.buildTree(predictedPoses);
		vector<bool> visitedPredictedPoses(predictedPoses.size(),false);
		/*get rid the agents' speed are too slow*/
		for (size_t i = 0;i < predictedPoses.size();i++){
			if (abs(agents_[i]->prefVelocity_) < adaptedPreferSpeedSlot){
				visitedPredictedPoses[i] = true;
			}
		}
		vector<size_t> centerID;
		
		while (1){
			vector<bool>::iterator iv = find(visitedPredictedPoses.begin(),visitedPredictedPoses.end(),false);
			if (visitedPredictedPoses.end() == iv){
				break;
			}
			size_t agentID = iv - visitedPredictedPoses.begin();

			visitedPredictedPoses[agentID] = true;
			
			Vector2 visitingPos = predictedPoses[agentID];

			vector<size_t> clusteredID;
			clusteredID.push_back(agentID);
			/*calculate the neighboring agents nearby cur agent*/
			float dst = 0;
			Vector2 pt = agents_[agentID]->searchNearest(visitingPos,dst);
			vector<skeletonPoint>::iterator ip = find(sltPts_.begin(),sltPts_.end(),pt);
			if (ip == sltPts_.end()){
				int ttt = 0;
			}
			size_t pID = ip - sltPts_.begin();
			centerID.push_back(pID);
			//clusterCirclesttt.push_back(*ip);
			vector<Vector2> clusteredPoses = predictedPosesTree.searchNearestRange(ip->position,ip->ran);
			for (size_t i = 0;i < clusteredPoses.size();i++){
				/*find the agents ID in cluster and put them in to clusteredID*/
				for (size_t agentID = 0;agentID < predictedPoses.size();agentID++){
					if (predictedPoses[agentID] == clusteredPoses[i]){
						if (false == visitedPredictedPoses[agentID]){
							visitedPredictedPoses[agentID] = true;
							clusteredID.push_back(agentID);
						}
					}
				}
			}
			/*calculate the density*/
			float den  = densityThreshold - 0.001;
			if (ip->ran != 0){
				den = clusteredID.size()*RadiusOfRobot / ip->ran;
			}
			/*push them to a cluster*/
			clusteredAgentsIDByMappingPoses.push_back(clusteredID);
			density.push_back(den);
		}
		return centerID;
		
	}
	void RVOSimulator::desityEqualization(){
		vector<Vector2> predictedPoses = predictedPoses_;
		while(1){
			vector<float> density;
			vector< vector<size_t> > clusteredAgentsIDByMappingPoses;
			vector<size_t> centersID = clusterMappingPositionsOfAgents(density, predictedPoses, clusteredAgentsIDByMappingPoses);
			/*check if need to equalizations the density*/
			bool equalization = false;
			for (size_t i = 0;i < density.size();i++){
				if (density[i] > densityThreshold){
					equalization = true;
				}
			}
			if (false == equalization){// no deed to do Equalization
				return;
			}			
			for (size_t i = 0;i < centersID.size();i++){	
				skeletonPoint center = sltPts_[centersID[i]];
				/*find the clustered agents around a center */
				vector<Vector2> clusteredAgentsPredictedPoses;
				
				for (size_t r = 0;r < clusteredAgentsIDByMappingPoses[i].size();r++){
					size_t ID = clusteredAgentsIDByMappingPoses[i][r];
					clusteredAgentsPredictedPoses.push_back(predictedPoses[ID]);
				}
				int ID0 = clusteredAgentsIDByMappingPoses[i][0];
				Vector2 generalMovingDir = agents_[ID0]->prefVelocity_;//define the general moving direction
				
				size_t nAgentInCluster = clusteredAgentsPredictedPoses.size();
				
				/*calculate how many agents need to slow down*/
				 int nflow = 1 + (nAgentInCluster*RadiusOfRobot - densityThreshold * center.ran) / RadiusOfRobot;
				/*sort the agents distant according to their moving directions and positions*/

				vector<pair<size_t,float> > distSortDir, distSortInverseDir;
				for (size_t j = 0;j < clusteredAgentsIDByMappingPoses[i].size();j++){
					int agentID = clusteredAgentsIDByMappingPoses[i][j];
					float angle = agents_[agentID]->velocity_ * generalMovingDir;
					float dst = abs(clusteredAgentsPredictedPoses[j] - center.position)* sgn(angle);
					if (angle > 0){
						size_t iinsterPos = distSortDir.size();
						for (size_t k = 0;k < distSortDir.size();k++){
							if (dst < distSortDir[k].second){
								iinsterPos = k;
								break;
							}
						}
						distSortDir.insert(distSortDir.begin() + iinsterPos, make_pair(agentID,dst));
					}
					else{
						size_t iinsterPos = distSortInverseDir.size();
						for (size_t k = 0;k < distSortInverseDir.size();k++){
							if (dst < distSortInverseDir[k].second){
								iinsterPos = k;
								break;
							}
						}
						distSortInverseDir.insert(distSortInverseDir.begin() + iinsterPos, make_pair(agentID,dst));
					}
				}
				/*build equalization sort */
				size_t smallerSortSize = distSortDir.size() > distSortInverseDir.size()
					?distSortInverseDir.size():distSortDir.size();
				vector<pair<size_t,float> > agentsSort;
				for (int d = 0;d < smallerSortSize;d++){
					if (d%2==0&&!distSortDir.empty()){
						agentsSort.push_back(distSortDir[d]);
					}
					if (d%2==1&&!distSortInverseDir.empty()){
						agentsSort.push_back(distSortInverseDir[d]);
					}
				}
				/*insertThe rest sort*/
				if (distSortDir.size() > distSortInverseDir.size()){
					size_t offset = distSortInverseDir.size();
					agentsSort.insert(agentsSort.end(),distSortDir.begin()+offset,distSortDir.end());
				} 
				else{
					size_t offset = distSortDir.size();
					agentsSort.insert(agentsSort.end(),distSortInverseDir.begin()+offset,distSortInverseDir.end());
				}
				/*slow the agents according to the sorting result*/
			
				for (int d = 0;d < nflow;d++){
					/*find the poses id of postpone agent*/
					float dst = agentsSort[d].second;//predicted pose ID
					int aID = agentsSort[d].first;
					agents_[aID]->activeAdaptedSpeed(maxPredictFrame);
					Vector2 moveOffset = agents_[aID]->prefVelocity_ * maxPredictFrame;
					Vector2 ap = agents_[aID]->position_ + moveOffset;
					Vector2 newPredictedPos = agents_[aID]->searchNearest(ap,dst);
					//cvCircle(img, cvPoint(predictedPoses[aID].x(), predictedPoses[aID].y()), RadiusOfRobot,cvScalar(0,0,0),2);					
					predictedPoses[aID] = newPredictedPos;
 				}
			}
			if (false)
				for (size_t i = 0; i < predictedPoses.size(); ++i){
					cout<<predictedPoses[i].x()<<" "<<predictedPoses[i].y()<<endl;
				}

			
			// height , width
			int width = 500;
			int bin_width = width/density.size();
			int bin_height = 400;
			cv::Mat hist=cv::Mat::zeros(bin_height,width,CV_8UC3);
			cout<<density.size()<<endl;

			for (int i=0; i<density.size(); ++i){
				cout<<density[i]<<" ";
				cv::rectangle(hist,cv::Point(i*bin_width,bin_height),
					cv::Point((i+1)*bin_width-5,bin_height-bin_height*density[i]),cv::Scalar(255,255,255), -1, 8, 0);
			}
			cv::line(hist,cv::Point(0,bin_height*(1-densityThreshold)),
				cv::Point(width,bin_height*(1-densityThreshold)), cv::Scalar(0,0,255),2,8,0);
			cout<<endl;
			imshow("hist",hist);
		}
	}
}

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(vector<int> dist, vector<bool> sptSet)
{
	// Initialize min value
	int min = INT_MAX, min_index = -1;

	for (int v = 0; v < dist.size(); v++)
		if (sptSet[v] == false && dist[v] <= min && dist[v] != -1)
			min = dist[v], min_index = v;

	return min_index;
}



// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
vector<size_t>  dijkstra(vector< vector<int> > initialDst, int src)
{
	size_t nNodes = initialDst.size();
	vector<int> dist(nNodes,-1);     // The output array.  dist[i] will hold the shortest
	// distance from src to i

	vector<bool> sptSet(nNodes,false); // sptSet[i] will true if vertex i is included in shortest
	// path tree or shortest distance from src to i is finalized


	// Distance of source vertex from itself is always 0
	dist[src] = 0;
	vector<size_t> shortestPaths(nNodes,src);
	// Find shortest path for all vertices
	for (int count = 0; count < nNodes-1; count++){
		// Pick the minimum distance vertex from the set of vertices not
		// yet processed. u is always equal to src in first iteration.
		int u = minDistance(dist, sptSet);
	
		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the picked vertex.
		for (int v = 0; v < nNodes; v++){
			// Update dist[v] only if is not in sptSet, there is an edge from 
			// u to v, and total weight of path from src to  v through u is 
			// smaller than current value of dist[v]
			if (!sptSet[v] && initialDst[u][v]!= -1 && dist[u] != -1 
				&& (dist[u]+initialDst[u][v] < dist[v] ||  dist[v] == -1)){
					dist[v] = dist[u] + initialDst[u][v];
					shortestPaths[v] = u;
			}
		}
	}
	return shortestPaths;
}
