#include "Robot.h"
#include "Definitions.h"
const size_t NCONNECT = 6;
const float agentRadius = 20;
typedef int AgentId;
class Link{
public:
	Link(){}
	Link(size_t data_, size_t next_):data(data_),next(next_){
	}
	size_t data;
	size_t next;
	bool operator ==(size_t val){
		if (val == data){
			return true;
		} 
		else{
			return false;
		}
	}
};


class Element{
public:
	AgentId occupiedAgentID; // if it is not occupied then is -1;
	//size_t ID;
	size_t loopID;
	size_t clusterID;
	float r;//radius
	Vector2 position;
	vector<size_t> connectionsElementsID;// the elements ID connect with this element. 
};
class Loop{
public:
	vector<Link> elementsID;
	
	//size_t clusterID;
};
class Cluster{
public:
	Vector2 v;//center of circular area
	float ran;//radius of circular area
	vector<Loop> loops;
	vector<size_t> connectionsClustersID;// the clusters ID connect with this cluster. 
};

class agentMovement
{
public:
	size_t ID;
	Vector2 subGoal;
};
class AgentsSequenceMovements{
public:
	vector<vector<agentMovement>>  agentMovement;
};
class Simulator
{
public:
	Simulator();//example
		
	Simulator(vector< vector<Element> > elements){
		 intialElements(elements);
	}
	void intialElements(vector< vector<Element> > elements);

	AgentsSequenceMovements  Simulator::nearbyElementsExchange(size_t cID0, size_t cID1);

	

	AgentsSequenceMovements  Simulator::intraLoopElementsExchange(size_t cID0, size_t cID1);
	
	AgentsSequenceMovements  Simulator::interLoopElementsExchange(size_t cID0, size_t cID1);
	
	AgentsSequenceMovements  Simulator::interClusterElementsExchange(size_t cID0, size_t cID1);
	void setElementPosition(size_t ID, Vector2 position){
		elements_[ID].position = position;
	}
	vector<Element> getElements(){
		return elements_;
	}

	//push elements assume cID0 and cID1 in clockwise order.
	AgentsSequenceMovements  rotateElementsInSameLoop(size_t cID0, size_t cID1, size_t steps, bool clockWise); 
private:
	
	vector<size_t> getElementsIdBetweenTwoElements(size_t cID0, size_t cID1);//clockwise
	void changeTheConnectionsAfterTwoElementsExchangeTheirPositions(size_t cID0, size_t cID1);
	void example();
	void benchmark1();
private:

	vector<size_t> agent_;//the indexs of agents associate with element ID
	vector<Cluster> clusters_;
	vector<Element> elements_;
};
