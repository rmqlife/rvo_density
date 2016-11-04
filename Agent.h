/*
 * Agent.h
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

#ifndef RVO_AGENT_H_
#define RVO_AGENT_H_

/**
 * \file       Agent.h
 * \brief      Contains the Agent class.
 */

#include "Definitions.h"
#include "RVOSimulator.h"

namespace RVO {
	/**
	 * \brief      Defines an agent in the simulation.
	 */
	class Agent {
	private:
		/**
		 * \brief      Constructs an agent instance.
		 * \param      sim             The simulator instance.
		 */
		explicit Agent(RVOSimulator *sim);

		/**
		 * \brief      Computes the neighbors of this agent.
		 */
		void computeNeighbors();

		/**
		 * \brief      Computes the new velocity of this agent.
		 */
		void computeNewVelocity();

		/**
		 * \brief      Inserts an agent neighbor into the set of neighbors of
		 *             this agent.
		 * \param      agent           A pointer to the agent to be inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertAgentNeighbor(const Agent *agent, float &rangeSq);

		/**
		 * \brief      Inserts a static obstacle neighbor into the set of neighbors
		 *             of this agent.
		 * \param      obstacle        The number of the static obstacle to be
		 *                             inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq);

		/**
		 * \brief      Updates the two-dimensional position and two-dimensional
		 *             velocity of this agent.
		 */
		void update();

		std::vector<std::pair<float, const Agent *> > agentNeighbors_;
		size_t maxNeighbors_;
		float maxSpeed_;
		float neighborDist_;
		Vector2 newVelocity_;
		std::vector<std::pair<float, const Obstacle *> > obstacleNeighbors_;
		std::vector<Line> orcaLines_;
		Vector2 position_;
		Vector2 prefVelocity_;
		float radius_;
		RVOSimulator *sim_;
		float timeHorizon_;
		float timeHorizonObst_;
		Vector2 velocity_;

		size_t id_;

		
		friend class KdTree;
		friend class RVOSimulator;

		//////////////////
	public:
		vector<Vector2> subGoals;
		Vector2 goal;
		size_t nextGoalID;
		void setSubGoals(vector<Vector2> subGoals_){
			subGoals = subGoals_;
			nextGoalID = 0;
		}
		int getNextGoalID(){
			if (nextGoalID == subGoals.size()-1){
				return -1; //done
			}
			return ++nextGoalID;
		}
		Vector2 getGoal(){
			return goal;
		}
		/*kd tree for calculating the mapping position*/
		kdTreeGeneral pathTree;
		void buildTree(vector<Vector2> data){
			pathTree.buildTree(data);
		}
		Vector2 searchNearest(Vector2 pt, float &dst){
			return pathTree.searchNearest(pt,dst);
		}
		/*adapted the agent maxSpeed to avoid high density situations*/
		int adaptedCount;
		void activeAdaptedSpeed(int equalizationCount){			
			adaptedCount = equalizationCount;
			Vector2 npre = prefVelocity_;
			npre = normalize(npre);
			
			if ( std::abs(abs(prefVelocity_) - adaptedPreferSpeedSlot) > 0){
				prefVelocity_ = npre * (abs(prefVelocity_) - adaptedPreferSpeedSlot);
			}
			else{
				prefVelocity_ = npre * (adaptedPreferSpeedSlot-0.001);
			}
		}
		void setAdaptedPreferVeloicty(){
			//Vector2 npre = normalize(subGoals[nextGoalID] - position_);
			//npre = normalize(npre);
			if (prefVelocity_.y() < -0.5){
				//IplImage* img = cvLoadImage( fileName.c_str() );
				////for (size_t i = 0;i<subGoals.size();i++){
				//	cvCircle(img, cvPoint(subGoals[nextGoalID].x(), subGoals[nextGoalID].y()),
				//		RadiusOfRobot,cvScalar(0,255,0),3);
				////}
				//cvCircle(img, cvPoint(position_.x(), position_.y()),
				//	RadiusOfRobot,cvScalar(0,0,255),3);
				//cvShowImage("",img);
				//cvWaitKey(NULL);
				int ttt = 0;
			}
			Vector2 refrePrefDir = normalize(subGoals[nextGoalID] - position_);
			/*visualize*/
			
			if (abs(position_-getGoal()) <= 6){
				prefVelocity_ = Vector2(0,0);
				return;
			}
			
			if(0 == adaptedCount ){//recovery
				if((abs(prefVelocity_) + 1.0/maxPredictFrame) < maxSpeed_ ){
					prefVelocity_ = refrePrefDir * (abs(prefVelocity_) + 1.0/maxPredictFrame);
				}
				else{
					prefVelocity_ = refrePrefDir * maxSpeed_;
				}
			}
			else{
				adaptedCount--;
			}
		
			
		}
	};

	/**
	 * \relates    Agent
	 * \brief      Solves a one-dimensional linear program on a specified line
	 *             subject to linear constraints defined by lines and a circular
	 *             constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      lineNo        The specified line constraint.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     True if successful.
	 */
	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo,
						float radius, const Vector2 &optVelocity,
						bool directionOpt, Vector2 &result);

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     The number of the line it fails on, and the number of lines if successful.
	 */
	size_t linearProgram2(const std::vector<Line> &lines, float radius,
						  const Vector2 &optVelocity, bool directionOpt,
						  Vector2 &result);

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      numObstLines  Count of obstacle lines.
	 * \param      beginLine     The line on which the 2-d linear program failed.
	 * \param      radius        The radius of the circular constraint.
	 * \param      result        A reference to the result of the linear program.
	 */
	void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine,
						float radius, Vector2 &result);
}

#endif /* RVO_AGENT_H_ */
